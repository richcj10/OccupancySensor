#include "EEPROMData.h"
#include <EEPROM.h>

char GetAddressFromEEPROM(){
    char ReturnVar = 0;
    char error = 0;
    //char result = eeprom_crc();
/*     if(result == EEPROM.read(CRC)){
        error++;
    } */
    ReturnVar = EEPROM.read(ADDRESS);
    if(error == 0){
        return ReturnVar;
    }
    else{
        return -1;
    }
}

char SetAddressFromEEPROM(char NewAddress){
    char error = 0;
    EEPROM.write(ADDRESS, NewAddress);
    if(EEPROM.read(ADDRESS) == NewAddress){
        error++;
    }
    if(error == 0){
        //EEPROM.write(CRC, eeprom_crc());
    }
    return error;
}

int GetScanRateFromEEPROM(){
    int ReturnVar = 0;
    char error = 0;
    //char result = eeprom_crc();
    //if(result == EEPROM.read(CRC)){
    //    error++;
    //}
    char HByte = EEPROM.read(SAMPLERATE);
    char LByte = EEPROM.read(SAMPLERATE+1);
    ReturnVar = (HByte<<8) | LByte;
    if(error == 0){
        return ReturnVar;
    }
    else{
        return -1;
    }
}

char SetScanRateFromEEPROM(unsigned int NewRate){
    char error = 0;
    unsigned char HByte = (unsigned char)((NewRate >> 0x08) & 0xff);
    unsigned char LByte = (unsigned char)(NewRate & 0xff);
    EEPROM.write(SAMPLERATE,HByte);
    EEPROM.write(SAMPLERATE+1,LByte);
    if(EEPROM.read(SAMPLERATE) == HByte){
        error++;
    }
    if(EEPROM.read(SAMPLERATE+1) == LByte){
        error++;
    }
    if(error == 0){
        //EEPROM.write(CRC, eeprom_crc());
    }
    return error;
}

unsigned long eeprom_crc(void) {

  const unsigned long crc_table[16] = {
    0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
    0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
    0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
    0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
  };

  unsigned long crc = ~0L;

  for (unsigned int index = 1 ; index < EEPROM.length()  ; ++index) {
    crc = crc_table[(crc ^ EEPROM[index]) & 0x0f] ^ (crc >> 4);
    crc = crc_table[(crc ^ (EEPROM[index] >> 4)) & 0x0f] ^ (crc >> 4);
    crc = ~crc;
  }
  return crc;
}