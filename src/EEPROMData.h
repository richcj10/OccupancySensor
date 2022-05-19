#ifndef EEPROMDATA_H
#define EEPROMDATA_H

#define CRC 0x00
#define ADDRESS 0x01
#define SAMPLERATE 0x02 //+1

char GetAddressFromEEPROM();
char SetAddressFromEEPROM(char NewAddress);
int GetScanRateFromEEPROM();
char SetScanRateFromEEPROM(unsigned int NewRate);
unsigned long eeprom_crc(void);

#endif