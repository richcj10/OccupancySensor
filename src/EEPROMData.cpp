#include "EEPROMData.h"
#include <Arduino.h>
#include <EEPROM.h>

char GetScanRateFromEEPROM() {
    uint8_t val = EEPROM.read(SAMPLERATE);
    return (val == 0xFF) ? -1 : (char)val;
}

char SetScanRateFromEEPROM(char NewRate) {
    EEPROM.update(SAMPLERATE, (uint8_t)NewRate);
    return (EEPROM.read(SAMPLERATE) == (uint8_t)NewRate) ? 1 : -1;
}
