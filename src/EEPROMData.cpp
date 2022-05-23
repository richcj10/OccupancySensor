#include "EEPROMData.h"
#include <Arduino.h>
#include <EEPROM.h>

union {
   unsigned long position;
   unsigned char bytes[4];
} CurrentPosition;

char GetAddressFromEEPROM(){
    char ReturnVar = 0;
    if(CheckCRC()){
        ReturnVar = EEPROM.read(ADDRESS);
        return ReturnVar;
    }
    else{
        return -1;
    }
}

char SetAddressFromEEPROM(char NewAddress){
    char error = 0;
    EEPROM.write(ADDRESS, NewAddress);
    if(EEPROM.read(ADDRESS) != NewAddress){
        error++;
    }
    if(error == 0){
        unsigned long crcReturn = eeprom_crc();
        CurrentPosition.position = crcReturn;
        EEPROM.write(EEPROM.length()-4, CurrentPosition.bytes[3]);
        EEPROM.write(EEPROM.length()-3, CurrentPosition.bytes[2]);
        EEPROM.write(EEPROM.length()-2, CurrentPosition.bytes[1]);
        EEPROM.write(EEPROM.length()-1, CurrentPosition.bytes[0]);
        return 1;
    }
    return error;
}

char GetScanRateFromEEPROM(){
    char ReturnVar = 0;
    if(CheckCRC()){
        ReturnVar = EEPROM.read(SAMPLERATE);
        return ReturnVar;
    }
    else{
        return -1;
    }
}

char SetScanRateFromEEPROM(char NewRate){
    char error = 0;
    EEPROM.write(SAMPLERATE,NewRate);
    if(EEPROM.read(SAMPLERATE) != NewRate){
        error++;
    }
    if(error == 0){
        unsigned long crcReturn = eeprom_crc();
        CurrentPosition.position = crcReturn;
        EEPROM.write(EEPROM.length()-4, CurrentPosition.bytes[3]);
        EEPROM.write(EEPROM.length()-3, CurrentPosition.bytes[2]);
        EEPROM.write(EEPROM.length()-2, CurrentPosition.bytes[1]);
        EEPROM.write(EEPROM.length()-1, CurrentPosition.bytes[0]);
        return 1;
    }
    return -1;
}

unsigned long eeprom_crc(void) {

  const unsigned long crc_table[16] = {
    0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
    0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
    0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
    0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
  };

  unsigned long crc = ~0L;

  for (unsigned int index = 0 ; index < EEPROM.length() - 4; ++index) {
    crc = crc_table[(crc ^ EEPROM[index]) & 0x0f] ^ (crc >> 4);
    crc = crc_table[(crc ^ (EEPROM[index] >> 4)) & 0x0f] ^ (crc >> 4);
    crc = ~crc;
  }
  return crc;
}

unsigned long eeprom_crc_all(void) {

  const unsigned long crc_table[16] = {
    0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
    0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
    0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
    0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
  };

  unsigned long crc = ~0L;

  for (unsigned int index = 0 ; index < EEPROM.length(); ++index) {
    crc = crc_table[(crc ^ EEPROM[index]) & 0x0f] ^ (crc >> 4);
    crc = crc_table[(crc ^ (EEPROM[index] >> 4)) & 0x0f] ^ (crc >> 4);
    crc = ~crc;
    Serial.println(index);
  }
  return crc;
}

char CheckCRC(){
    unsigned long crcReturn = eeprom_crc();
    CurrentPosition.position = crcReturn;
    if( EEPROM.read(EEPROM.length()-4) == CurrentPosition.bytes[3]){
        if( EEPROM.read(EEPROM.length()-3) == CurrentPosition.bytes[2]){
            if( EEPROM.read(EEPROM.length()-2) == CurrentPosition.bytes[1]){
                if( EEPROM.read(EEPROM.length()-1) == CurrentPosition.bytes[0]){
                    return 1;
                }
                return 0;
            }
            return 0;
        }
        return 0;
    }
    return 0;
}