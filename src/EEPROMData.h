#ifndef EEPROMDATA_H
#define EEPROMDATA_H

#define CRC 0x00
#define ADDRESS 0x01
#define SAMPLERATE 0x02 //+1

char GetAddressFromEEPROM();
char SetAddressFromEEPROM(char NewAddress);
char GetScanRateFromEEPROM();
char SetScanRateFromEEPROM(char NewRate);
char CheckCRC();
unsigned long eeprom_crc(void);
unsigned long eeprom_crc_all(void);

#endif