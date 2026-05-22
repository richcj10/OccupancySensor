#!/usr/bin/env python3
"""
mbbp_flash.py — MBBP Firmware Flasher
======================================
Flash firmware to a ModBusBL2000 slave over RS-485.

Supports:
  • Intel HEX (.hex) — standard PlatformIO output
  • Raw binary  (.bin) — alternative PlatformIO output

Usage examples
--------------
  # Trigger bootloader and flash firmware:
  python mbbp_flash.py --port COM3 --slave 1 --firmware .pio/build/nanoatmega328/firmware.hex

  # Device already in bootloader mode (skip FC65 trigger):
  python mbbp_flash.py --port COM3 --slave 1 --firmware firmware.bin --no-trigger

  # Different baud rate:
  python mbbp_flash.py --port COM3 --slave 1 --baud 9600 --firmware firmware.hex

Requirements
------------
  pip install pyserial
  pip install intelhex   (only needed for .hex files)
"""

import argparse
import struct
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("ERROR: pyserial not installed. Run: pip install pyserial")

# ── MBBP constants ────────────────────────────────────────────────────────────
FC_TRIGGER    = 0x41
MAGIC_H       = 0xB0
MAGIC_L       = 0x07

CMD_HELLO      = 0x41
CMD_START      = 0x42
CMD_WRITE_PAGE = 0x43
CMD_VERIFY     = 0x44
CMD_JUMP       = 0x45

RSP_HELLO      = 0xC1
RSP_START      = 0xC2
RSP_WRITE_PAGE = 0xC3
RSP_VERIFY_OK  = 0xC4
RSP_JUMP       = 0xC5
RSP_ERROR      = 0xFF

PAGE_SIZE      = 128
MAX_PAGES      = 240      # 30 KB app space

ERROR_CODES = {
    0x01: "BAD_CRC",
    0x02: "WRONG_MAGIC",
    0x03: "PAGE_OVERFLOW",
    0x04: "FLASH_FAIL",
    0x05: "VERIFY_FAIL",
    0x06: "WRONG_STATE",
    0x07: "TIMEOUT",
}

# ── CRC-16 (Modbus: poly 0xA001, init 0xFFFF) ────────────────────────────────
def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 0x0001) else (crc >> 1)
    return crc & 0xFFFF


# ── Firmware loading ──────────────────────────────────────────────────────────
def load_firmware(path: str) -> bytes:
    """Load firmware from .hex or .bin file. Returns raw binary bytes."""
    if path.lower().endswith(".hex"):
        try:
            from intelhex import IntelHex
        except ImportError:
            sys.exit("ERROR: intelhex not installed. Run: pip install intelhex\n"
                     "       Or convert to .bin first:\n"
                     "       avr-objcopy -I ihex -O binary firmware.hex firmware.bin")
        ih = IntelHex()
        ih.loadhex(path)
        # Extract from address 0 to the last used address.
        start = ih.minaddr()
        end   = ih.maxaddr() + 1
        if start != 0:
            print(f"WARNING: firmware does not start at address 0 (starts at 0x{start:04X}).")
        raw = bytes(ih.tobinarray(start=0, end=end - 1))
        return raw
    else:
        with open(path, "rb") as f:
            return f.read()


# ── MBBP frame helpers ────────────────────────────────────────────────────────
def build_frame(addr: int, cmd: int, payload: bytes = b"") -> bytes:
    hdr = bytes([addr, cmd, len(payload) >> 8, len(payload) & 0xFF])
    body = hdr + payload
    crc  = crc16(body)
    return body + bytes([crc >> 8, crc & 0xFF])


def parse_frame(data: bytes, expected_addr: int):
    """
    Parse an MBBP response frame.
    Returns (cmd, payload) on success.
    Raises ValueError on error.
    """
    if len(data) < 6:
        raise ValueError(f"Frame too short ({len(data)} bytes)")
    if data[0] != expected_addr:
        raise ValueError(f"Wrong address: got 0x{data[0]:02X}, expected 0x{expected_addr:02X}")
    cmd     = data[1]
    pay_len = (data[2] << 8) | data[3]
    if len(data) != 4 + pay_len + 2:
        raise ValueError(f"Frame length mismatch: got {len(data)}, expected {4+pay_len+2}")
    rx_crc   = (data[-2] << 8) | data[-1]
    calc_crc = crc16(data[:-2])
    if rx_crc != calc_crc:
        raise ValueError(f"CRC mismatch: got 0x{rx_crc:04X}, calculated 0x{calc_crc:04X}")
    payload = data[4:4 + pay_len]
    if cmd == RSP_ERROR and pay_len >= 1:
        err = payload[0]
        name = ERROR_CODES.get(err, f"0x{err:02X}")
        raise ValueError(f"Slave returned ERROR: {name}")
    return cmd, payload


def recv_frame(port: serial.Serial, timeout: float = 2.0) -> bytes:
    """
    Read one complete MBBP frame determined by its 4-byte header length field.
    More reliable than gap-based reading — Windows sleep() resolution (~15 ms)
    is too coarse for a 5 ms gap check at 38400 baud.
    """
    deadline = time.monotonic() + timeout

    def read_exact(n: int) -> bytes:
        buf = b''
        while len(buf) < n:
            if time.monotonic() > deadline:
                raise TimeoutError(
                    f"Timeout reading frame (got {len(buf)}/{n} bytes)")
            waiting = port.in_waiting
            if waiting:
                buf += port.read(min(waiting, n - len(buf)))
            else:
                time.sleep(0.0005)  # 0.5 ms poll — well below Windows timer floor
        return buf

    header  = read_exact(4)
    pay_len = (header[2] << 8) | header[3]
    rest    = read_exact(pay_len + 2)   # payload + 2 CRC bytes
    return header + rest


# ── ModBus FC65 trigger ───────────────────────────────────────────────────────
def build_fc65(slave_id: int) -> bytes:
    """
    Build the 6-byte ModBus FC65 trigger frame.
    CRC storage matches SimpleModbusSlave: [crc_hi, crc_lo] — byte-swapped.
    """
    frame = bytes([slave_id, FC_TRIGGER, MAGIC_H, MAGIC_L])
    crc   = crc16(frame)
    return frame + bytes([crc & 0xFF, crc >> 8])  # ModBus: low byte first


def trigger_bootloader(port: serial.Serial, slave_id: int, verbose: bool) -> bool:
    """Send FC65 and wait for ACK. Returns True if ACK received."""
    if verbose:
        print(f"  Sending FC65 trigger to slave {slave_id}...")

    frame = build_fc65(slave_id)
    port.reset_input_buffer()
    port.write(frame)
    port.flush()

    try:
        rsp = recv_frame(port, timeout=1.0)
    except TimeoutError:
        print("  WARNING: No ACK from slave (device may already be in bootloader mode).")
        return False

    if len(rsp) >= 2 and rsp[0] == slave_id and rsp[1] == FC_TRIGGER:
        if verbose:
            print("  FC65 ACK received — device rebooting to bootloader.")
        return True

    print(f"  WARNING: Unexpected response to FC65: {rsp.hex()}")
    return False


# ── MBBP session ─────────────────────────────────────────────────────────────
def mbbp_connect(port: serial.Serial, slave_id: int, verbose: bool, retries: int = 5) -> bool:
    """Send HELLO and confirm bootloader is responsive."""
    payload = bytes([MAGIC_H, MAGIC_L])
    frame   = build_frame(slave_id, CMD_HELLO, payload)

    for attempt in range(1, retries + 1):
        if verbose:
            print(f"  HELLO attempt {attempt}/{retries}...")
        port.reset_input_buffer()
        port.write(frame)
        port.flush()
        try:
            rsp  = recv_frame(port, timeout=1.5)
            cmd, pay = parse_frame(rsp, slave_id)
            if cmd == RSP_HELLO:
                max_pages = (pay[0] << 8) | pay[1] if len(pay) >= 2 else 0
                if verbose:
                    print(f"  Bootloader ready. Available pages: {max_pages} ({max_pages * PAGE_SIZE // 1024} KB).")
                return True
        except (TimeoutError, ValueError) as e:
            if verbose:
                print(f"  {e}")
        time.sleep(0.3)

    return False


def mbbp_flash(port: serial.Serial, slave_id: int, firmware: bytes,
               verbose: bool, progress: bool) -> bool:
    """Transfer firmware, verify, and jump."""
    fw_crc   = crc16(firmware)
    fw_size  = len(firmware)
    n_pages  = (fw_size + PAGE_SIZE - 1) // PAGE_SIZE

    # ── START ──────────────────────────────────────────────────────────────
    pay = struct.pack(">IH", fw_size, fw_crc)
    port.reset_input_buffer()
    port.write(build_frame(slave_id, CMD_START, pay))
    port.flush()
    rsp = recv_frame(port, timeout=3.0)
    cmd, _ = parse_frame(rsp, slave_id)
    if cmd != RSP_START:
        print(f"ERROR: Expected RSP_START (0x{RSP_START:02X}), got 0x{cmd:02X}")
        return False
    if verbose:
        print(f"  START accepted. Sending {n_pages} pages ({fw_size} bytes)...")

    # ── WRITE_PAGE ─────────────────────────────────────────────────────────
    for pg in range(n_pages):
        offset = pg * PAGE_SIZE
        chunk  = firmware[offset: offset + PAGE_SIZE]
        if len(chunk) < PAGE_SIZE:
            chunk = chunk + bytes([0xFF] * (PAGE_SIZE - len(chunk)))  # pad

        pay = struct.pack(">H", pg) + chunk
        port.reset_input_buffer()
        port.write(build_frame(slave_id, CMD_WRITE_PAGE, pay))
        port.flush()

        rsp = recv_frame(port, timeout=3.0)
        cmd, rsp_pay = parse_frame(rsp, slave_id)
        if cmd != RSP_WRITE_PAGE:
            print(f"\nERROR: page {pg} rejected (cmd=0x{cmd:02X})")
            return False
        echo_page = (rsp_pay[0] << 8) | rsp_pay[1] if len(rsp_pay) >= 2 else -1
        if echo_page != pg:
            print(f"\nERROR: page echo mismatch (sent {pg}, got {echo_page})")
            return False

        if progress:
            pct = (pg + 1) * 90 // n_pages
            bar = "#" * (pct // 2) + "-" * (45 - pct // 2)
            print(f"\r  Flashing [{bar}] {pct}%  page {pg+1}/{n_pages}", end="", flush=True)

    if progress:
        print()  # newline after progress bar

    # ── VERIFY ─────────────────────────────────────────────────────────────
    if verbose:
        print(f"  Verifying (expected CRC 0x{fw_crc:04X})...")
    port.reset_input_buffer()
    port.write(build_frame(slave_id, CMD_VERIFY))
    port.flush()
    rsp = recv_frame(port, timeout=10.0)
    cmd, rsp_pay = parse_frame(rsp, slave_id)
    if cmd != RSP_VERIFY_OK:
        print(f"\nERROR: VERIFY rejected (cmd=0x{cmd:02X})")
        return False
    reported_crc = (rsp_pay[0] << 8) | rsp_pay[1] if len(rsp_pay) >= 2 else 0
    if verbose:
        print(f"  Slave CRC: 0x{reported_crc:04X} — {'OK' if reported_crc == fw_crc else 'MISMATCH!'}")

    # ── JUMP ───────────────────────────────────────────────────────────────
    if verbose:
        print("  Sending JUMP — device will start new firmware.")
    port.write(build_frame(slave_id, CMD_JUMP))
    port.flush()
    try:
        recv_frame(port, timeout=1.0)  # best-effort: device may jump before we read
    except TimeoutError:
        pass  # normal — device jumped immediately

    return True


# ── Entry point ───────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(
        description="Flash firmware to a ModBusBL2000 slave over RS-485 / MBBP.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)

    ap.add_argument("--port",       required=True,
                    help="Serial port (e.g. COM3, /dev/ttyUSB0)")
    ap.add_argument("--slave",      type=int, metavar="ID",
                    help="ModBus slave ID (1–247)")
    ap.add_argument("--firmware",   metavar="FILE",
                    help="Firmware file (.hex or .bin)")

    ap.add_argument("--scan", action="store_true",
                    help="Scan Modbus IDs on the bus")

    ap.add_argument("--set-id", type=int, metavar="NEW_ID",
                    help="Change device ID")

    ap.add_argument("--baud",       type=int, default=38400,
                    help="Serial baud rate (default: 38400)")
    ap.add_argument("--no-trigger", action="store_true",
                    help="Skip FC65 trigger (device already in bootloader mode)")
    ap.add_argument("--retries",    type=int, default=5,
                    help="HELLO connect retries after trigger (default: 5)")
    ap.add_argument("--timeout",    type=float, default=2.0,
                    help="Per-frame timeout in seconds (default: 2.0)")
    ap.add_argument("-v", "--verbose", action="store_true",
                    help="Verbose output")
    ap.add_argument("-q", "--quiet", action="store_true",
                    help="Suppress progress bar")

    args = ap.parse_args()

    if args.slave is not None:
        if args.slave < 1 or args.slave > 247:
            ap.error("--slave must be 1–247")


    # ── SCAN MODE ─────────────────────────────
    if args.scan:
        try:
            port = serial.Serial(
                port=args.port,
                baudrate=args.baud,
                bytesize=8,
                parity="N",
                stopbits=1,
                timeout=args.timeout)
        except serial.SerialException as e:
            sys.exit(f"ERROR opening {args.port}: {e}")

        print("Scanning Modbus IDs...")

        for sid in range(1, 248):
            try:
                frame = build_frame(sid, CMD_HELLO,
                                    bytes([MAGIC_H, MAGIC_L]))

                port.reset_input_buffer()
                port.write(frame)
                port.flush()

                rsp = recv_frame(port, timeout=0.15)
                cmd, _ = parse_frame(rsp, sid)

                if cmd == RSP_HELLO:
                    print(f"Found device at ID {sid}")

            except Exception:
                pass

        port.close()
        sys.exit(0)


    if not args.scan:
        if args.slave is None:
            ap.error("--slave is required unless using --scan")

        if args.firmware is None:
            ap.error("--firmware is required unless using --scan")

    # Load firmware
    print(f"Loading firmware from: {args.firmware}")
    try:
        firmware = load_firmware(args.firmware)
    except Exception as e:
        sys.exit(f"ERROR loading firmware: {e}")

    n_pages = (len(firmware) + PAGE_SIZE - 1) // PAGE_SIZE
    print(f"  {len(firmware)} bytes  ({n_pages} pages, CRC 0x{crc16(firmware):04X})")

    if n_pages > MAX_PAGES:
        sys.exit(f"ERROR: Firmware too large ({n_pages} pages > {MAX_PAGES} max).")

    # Open port
    try:
        port = serial.Serial(
            port=args.port, baudrate=args.baud,
            bytesize=8, parity="N", stopbits=1,
            timeout=args.timeout)
    except serial.SerialException as e:
        sys.exit(f"ERROR opening {args.port}: {e}")

    print(f"Port {args.port} open at {args.baud} baud.")

    try:
        # Step 1: trigger
        if not args.no_trigger:
            print(f"\n[1/3] Triggering bootloader on slave {args.slave}...")
            trigger_bootloader(port, args.slave, args.verbose)
            time.sleep(0.5)  # let device reboot
        else:
            print(f"\n[1/3] Skipping FC65 trigger (--no-trigger).")

        # Step 2: connect
        print(f"\n[2/3] Connecting to bootloader...")
        if not mbbp_connect(port, args.slave, args.verbose, retries=args.retries):
            sys.exit("ERROR: Could not connect to bootloader. Is the device in update mode?")

        # Step 3: flash
        print(f"\n[3/3] Flashing firmware...")
        success = mbbp_flash(port, args.slave, firmware,
                             verbose=args.verbose, progress=not args.quiet)
        if success:
            print("\nDone! Firmware update successful.")
        else:
            sys.exit("\nERROR: Firmware update failed.")

    finally:
        port.close()


if __name__ == "__main__":
    main()
