#ifndef EEPROMDATA_H
#define EEPROMDATA_H

// 0x00-0x04 reserved for ModBusBL / the bootloader (see MBBP_EE_* in MBBP.h):
//   boot flag, slave ID, app valid, sentinel, inverted slave ID
// Application data starts at 0x10
#define SAMPLERATE 0x10

char GetScanRateFromEEPROM();
char SetScanRateFromEEPROM(char NewRate);

#endif
