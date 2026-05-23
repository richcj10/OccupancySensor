#ifndef EEPROMDATA_H
#define EEPROMDATA_H

// 0x00-0x02 reserved for ModBusBL (boot flag, slave ID, app valid)
// Application data starts at 0x03
#define SAMPLERATE 0x03

char GetScanRateFromEEPROM();
char SetScanRateFromEEPROM(char NewRate);

#endif
