#include "ModBusBL.h"
#include "SimpleModbusSlave.h"
#include <EEPROM.h>
#include <avr/wdt.h>

ModBusBL *ModBusBL::_instance = nullptr;

ModBusBL::ModBusBL(uint8_t dirPin, uint8_t numHolding,
                   uint8_t numInput, uint8_t numCoil, uint8_t numDI)
    : _dirPin(dirPin),
      _numHolding(min(numHolding, (uint8_t)(MODBUSBL_MAX_HOLDING - 1))),
      _numInput  (min(numInput,   (uint8_t) MODBUSBL_MAX_INPUT)),
      _numCoil   (min(numCoil,    (uint8_t) MODBUSBL_MAX_COIL)),
      _numDI     (min(numDI,      (uint8_t) MODBUSBL_MAX_DI)),
      _slaveId(MBBP_SLAVE_ADDR),
      _errorCount(0)
{
    memset(_holding, 0, sizeof(_holding));
    memset(_input,   0, sizeof(_input));
    memset(_coil,    0, sizeof(_coil));
    memset(_di,      0, sizeof(_di));
}

void ModBusBL::begin(long baud) {
    /* Only one ModBusBL instance may exist (static callback limitation). */
    _instance = this;

    /* Load slave ID from EEPROM; initialise to default if unset (0xFF). */
    uint8_t stored = EEPROM.read(MBBP_EE_SLAVE_ID);
    if (stored >= 1 && stored <= 247)
        _slaveId = stored;
    else {
        _slaveId = MBBP_SLAVE_ADDR;
        EEPROM.write(MBBP_EE_SLAVE_ID, _slaveId);
    }

    /* Expose slave ID on holding register 0. */
    _holding[0] = _slaveId;

    modbus_register_bootloader_trigger(_bootloaderCallback);

    /* Total holding regs seen by ModBus = 1 (slave ID) + user regs. */
    modbus_configure(baud, _slaveId, _dirPin,
                     1 + _numHolding, _numInput, _numCoil, _numDI, 0);
}

void ModBusBL::update() {
    /* Keep reg 0 in sync with current slave ID (may have been changed). */
    _holding[0] = _slaveId;

    _errorCount = modbus_update(_holding, _input, _coil, _di);

    /* If the master wrote a new slave ID to reg 0, persist it. */
    uint16_t newId = _holding[0];
    if (newId >= 1 && newId <= 247 && (uint8_t)newId != _slaveId)
        setSlaveId((uint8_t)newId);
}

/* ── Holding registers ──────────────────────────────────────────────────── */
uint16_t ModBusBL::getHolding(uint8_t idx) const {
    if (idx < _numHolding) return _holding[idx + 1];
    return 0;
}

void ModBusBL::setHolding(uint8_t idx, uint16_t val) {
    if (idx < _numHolding) _holding[idx + 1] = val;
}

/* ── Input registers ────────────────────────────────────────────────────── */
uint16_t ModBusBL::getInput(uint8_t idx) const {
    if (idx < _numInput) return _input[idx];
    return 0;
}

void ModBusBL::setInput(uint8_t idx, uint16_t val) {
    if (idx < _numInput) _input[idx] = val;
}

/* ── Coils ──────────────────────────────────────────────────────────────── */
bool ModBusBL::getCoil(uint8_t idx) const {
    if (idx < _numCoil) return _coil[idx];
    return false;
}

void ModBusBL::setCoil(uint8_t idx, bool val) {
    if (idx < _numCoil) _coil[idx] = val;
}

/* ── Discrete inputs ────────────────────────────────────────────────────── */
bool ModBusBL::getDI(uint8_t idx) const {
    if (idx < _numDI) return _di[idx];
    return false;
}

void ModBusBL::setDI(uint8_t idx, bool val) {
    if (idx < _numDI) _di[idx] = val;
}

/* ── Slave ID ───────────────────────────────────────────────────────────── */
uint8_t ModBusBL::slaveId() const { return _slaveId; }

void ModBusBL::setSlaveId(uint8_t id) {
    if (id < 1 || id > 247) return;
    _slaveId   = id;
    _holding[0] = id;
    EEPROM.update(MBBP_EE_SLAVE_ID, id);
}

/* ── Bootloader trigger ─────────────────────────────────────────────────── */
void ModBusBL::_bootloaderCallback() {
    if (_instance) _instance->_triggerBootloader();
}

void ModBusBL::_triggerBootloader() {
    /* Write the boot-request flag so the bootloader knows why we rebooted. */
    EEPROM.write(MBBP_EE_BOOT_FLAG, MBBP_BOOT_FLAG_VAL);
    EEPROM.write(MBBP_EE_APP_VALID, 0xFF); /* clear valid flag during update */

    /* Short delay so the ACK frame clears the RS-485 bus before reset. */
    delay(20);

    /* Reset via watchdog — BOOTRST fuse sends execution to bootloader. */
    wdt_enable(WDTO_15MS);
    while (1) { /* spin until WDT fires */ }
}
