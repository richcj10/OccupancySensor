#ifndef MODBUS_BL_H
#define MODBUS_BL_H

/*
 * ModBusBL — portable ModBus slave library with bootloader support.
 *
 * Drop this library into any PlatformIO project.
 * Call begin() in setup(), update() in loop().
 *
 * Reserved holding register layout (master-visible):
 *   Reg 0  : Slave ID  (read = current ID, write = new ID saved to EEPROM)
 *   Reg 1+ : User application registers
 *
 * Bootloader entry:
 *   Master sends FC65 (0x41) with 2-byte magic payload 0xB0 0x07.
 *   Device ACKs, writes boot flag to EEPROM, then resets via WDT.
 *   Bootloader starts, sees flag, enters MBBP firmware-update mode.
 *
 * Register count limits (override before #include if needed):
 *   MODBUSBL_MAX_HOLDING  — total holding regs including reg 0 (default 17)
 *   MODBUSBL_MAX_INPUT    — input registers                    (default 16)
 *   MODBUSBL_MAX_COIL     — coil bits                          (default 8)
 *   MODBUSBL_MAX_DI       — discrete input bits                (default 8)
 */

#include "Arduino.h"
#include "MBBP.h"

#ifndef MODBUSBL_MAX_HOLDING
#define MODBUSBL_MAX_HOLDING  17   /* reg 0 = slave ID, regs 1-16 = user */
#endif
#ifndef MODBUSBL_MAX_INPUT
#define MODBUSBL_MAX_INPUT    16
#endif
#ifndef MODBUSBL_MAX_COIL
#define MODBUSBL_MAX_COIL     8
#endif
#ifndef MODBUSBL_MAX_DI
#define MODBUSBL_MAX_DI       8
#endif

class ModBusBL {
public:
    /*
     * dirPin   : RS-485 TX-enable pin (pulled HIGH to transmit).
     *            Set < 2 to disable (e.g. if hardware auto-handles direction).
     * numHolding: number of USER holding registers (reg 1 … numHolding).
     *             Must be <= MODBUSBL_MAX_HOLDING - 1.
     * numInput, numCoil, numDI: register counts, must fit within MAX defines.
     */
    ModBusBL(uint8_t dirPin     = MBBP_DIR_PIN,
             uint8_t numHolding = MODBUSBL_MAX_HOLDING - 1,
             uint8_t numInput   = MODBUSBL_MAX_INPUT,
             uint8_t numCoil    = MODBUSBL_MAX_COIL,
             uint8_t numDI      = MODBUSBL_MAX_DI);

    /* Reads slave ID from EEPROM; writes MBBP_SLAVE_ADDR as default if unset. */
    void begin(long baud = 38400);

    /* Call every iteration of loop(). Processes one ModBus frame if available. */
    void update();

    /* ── User holding registers (idx 0 = reg 1 on the bus) ────────────────── */
    uint16_t getHolding(uint8_t idx) const;
    void     setHolding(uint8_t idx, uint16_t val);

    /* ── Input registers (read-only from master, writable by app) ─────────── */
    uint16_t getInput(uint8_t idx) const;
    void     setInput(uint8_t idx, uint16_t val);

    /* ── Coils (readable and writable by master) ──────────────────────────── */
    bool getCoil(uint8_t idx) const;
    void setCoil(uint8_t idx, bool val);

    /* ── Discrete inputs (read-only from master, writable by app) ─────────── */
    bool getDI(uint8_t idx) const;
    void setDI(uint8_t idx, bool val);

    /* ── Slave ID ─────────────────────────────────────────────────────────── */
    uint8_t slaveId() const;
    void    setSlaveId(uint8_t id); /* saves to EEPROM immediately */

    /* Returns total ModBus error count since begin(). */
    unsigned int errorCount() const { return _errorCount; }

private:
    uint8_t  _dirPin;
    uint8_t  _numHolding;   /* user registers (excludes reg 0) */
    uint8_t  _numInput;
    uint8_t  _numCoil;
    uint8_t  _numDI;
    uint8_t  _slaveId;
    unsigned int _errorCount;

    /* Register storage: holding[0] mirrors slave ID on the bus */
    uint16_t _holding[MODBUSBL_MAX_HOLDING];
    uint16_t _input  [MODBUSBL_MAX_INPUT];
    bool     _coil   [MODBUSBL_MAX_COIL];
    bool     _di     [MODBUSBL_MAX_DI];

    void _triggerBootloader();

    /* Static trampoline so we can pass a plain function pointer to the forked
       SimpleModbusSlave while still calling an instance method. */
    static ModBusBL *_instance;
    static void _bootloaderCallback();
};

#endif /* MODBUS_BL_H */
