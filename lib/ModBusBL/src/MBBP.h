#ifndef MBBP_H
#define MBBP_H

/*
 * MBBP — ModBus Bootloader Protocol
 *
 * Custom binary protocol for firmware transfer over RS-485.
 * Shared by: ModBusBL (slave library), ModBusBLMaster (ESP32 master),
 *            bootloader (AVR), and mbbp_flash.py (Python tool).
 *
 * Frame format (both directions):
 *   [ADDR 1B][CMD 1B][LEN_H 1B][LEN_L 1B][DATA 0-130B][CRC_H 1B][CRC_L 1B]
 *
 * CRC is Modbus CRC-16 (poly 0xA001, init 0xFFFF) over all bytes
 * from ADDR through end of DATA.
 *
 * The slave ADDR byte is the device's ModBus slave ID (from EEPROM).
 * Other devices on the bus ignore frames not matching their slave ID.
 */

/* ── Modbus FC65 trigger (app → bootloader handoff) ─────────────────────── */
#define MBBP_FC_TRIGGER         0x41    /* FC65: sent via normal ModBus */
#define MBBP_MAGIC_H            0xB0
#define MBBP_MAGIC_L            0x07    /* magic word 0xB007 = "BOOT" */

/* ── MBBP session commands (master → slave) ──────────────────────────────── */
#define MBBP_CMD_HELLO          0x41    /* Start session, payload: [MAGIC_H][MAGIC_L] */
#define MBBP_CMD_START          0x42    /* Begin update, payload: [SIZE 4B][EXPECTED_CRC 2B] */
#define MBBP_CMD_WRITE_PAGE     0x43    /* Write flash page, payload: [PAGE# 2B][128B data] */
#define MBBP_CMD_VERIFY         0x44    /* Verify full image CRC, no payload */
#define MBBP_CMD_JUMP           0x45    /* Jump to app, no payload */

/* ── MBBP responses (slave → master) — CMD | 0x80 ───────────────────────── */
#define MBBP_RSP_HELLO          0xC1    /* payload: [MAX_PAGES 2B] */
#define MBBP_RSP_START          0xC2    /* no payload */
#define MBBP_RSP_WRITE_PAGE     0xC3    /* payload: [PAGE# 2B] (echo) */
#define MBBP_RSP_VERIFY_OK      0xC4    /* payload: [COMPUTED_CRC 2B] */
#define MBBP_RSP_JUMP           0xC5    /* no payload (device jumps immediately after) */
#define MBBP_RSP_ERROR          0xFF    /* payload: [ERROR_CODE 1B] */

/* ── Error codes ─────────────────────────────────────────────────────────── */
#define MBBP_ERR_BAD_CRC        0x01
#define MBBP_ERR_WRONG_MAGIC    0x02
#define MBBP_ERR_PAGE_OVERFLOW  0x03
#define MBBP_ERR_FLASH_FAIL     0x04
#define MBBP_ERR_VERIFY_FAIL    0x05
#define MBBP_ERR_WRONG_STATE    0x06
#define MBBP_ERR_TIMEOUT        0x07

/* ── Flash geometry (ATmega328P) ─────────────────────────────────────────── */
#define MBBP_PAGE_SIZE          128     /* bytes per flash page */
#define MBBP_APP_START          0x0000  /* byte address of app section start */
#define MBBP_APP_SIZE_MAX       30720   /* 30KB = 240 pages */
#define MBBP_MAX_PAGES          (MBBP_APP_SIZE_MAX / MBBP_PAGE_SIZE) /* 240 */

/* ── EEPROM layout ───────────────────────────────────────────────────────── */
#define MBBP_EE_BOOT_FLAG       0x00    /* 0xB0 = enter bootloader, 0xFF = normal */
#define MBBP_EE_SLAVE_ID        0x01    /* Modbus slave ID (1-247) */
#define MBBP_EE_APP_VALID       0x02    /* 0xA5 = valid app exists, else invalid */
/* 0x03 reserved for application scan rate */
#define MBBP_EE_SLAVE_INIT      0x04    /* sentinel: 0xA5 = SLAVE_ID was intentionally set */

#define MBBP_BOOT_FLAG_VAL      0xB0
#define MBBP_APP_VALID_VAL      0xA5
#define MBBP_SLAVE_INIT_VAL     0xA5    /* written alongside MBBP_EE_SLAVE_ID by setSlaveId() */
#define MBBP_ADDR_CHANGE_KEY    0xA5B0  /* master writes this to holding reg 1 alongside new ID in reg 0 (FC16) */

/* Default slave address — override per board via -DMBBP_SLAVE_ADDR=n in build_flags */
#ifndef MBBP_SLAVE_ADDR
#define MBBP_SLAVE_ADDR         1
#endif

/* ── Frame size constants ─────────────────────────────────────────────────── */
#define MBBP_HEADER_SIZE        4       /* ADDR + CMD + LEN_H + LEN_L */
#define MBBP_CRC_SIZE           2
#define MBBP_OVERHEAD           (MBBP_HEADER_SIZE + MBBP_CRC_SIZE)
#define MBBP_MAX_PAYLOAD        (2 + MBBP_PAGE_SIZE) /* page# + 128 data bytes */
#define MBBP_MAX_FRAME          (MBBP_OVERHEAD + MBBP_MAX_PAYLOAD) /* 136 bytes */

/* ── RS-485 default DIR pin ──────────────────────────────────────────────── */
#ifndef MBBP_DIR_PIN
#define MBBP_DIR_PIN            8       /* override via build_flags */
#endif

#endif /* MBBP_H */
