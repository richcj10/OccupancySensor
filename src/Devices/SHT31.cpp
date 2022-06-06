#include <Arduino.h>
#include <Wire.h>
#include "SHT31.h"

float temp = 0;
float humidity = 0;
char Address = 0;

void SHT31Start(char A) {
    Address = A;
}

void SHT31Reset(void) {
  WriteCommand(SHT31_SOFTRESET);
  delay(10);
}

void SHT31heater(bool h) {
  if (h)
    WriteCommand(SHT31_HEATEREN);
  else
    WriteCommand(SHT31_HEATERDIS);
  delay(1);
}

unsigned int ReadStatus(void) {
  WriteCommand(SHT31_READSTATUS);

  unsigned char data[3];
  ReadBytes(3,data);

  uint16_t stat = data[0];
  stat <<= 8;
  stat |= data[1];
  // Serial.println(stat, HEX);
  return stat;
}

bool SHT31HeaterEnabled() {
  uint16_t regValue = ReadStatus();
  return (bool)bitRead(regValue, SHT31_REG_HEATER_BIT);
}

bool WriteCommand(unsigned int command) {
  Wire.beginTransmission(Address);
  Wire.write(command >> 8 );
  Wire.write(command & 0xFF);
  if (Wire.endTransmission() != 0)
  {
    return false;
  }
  return true;
}

bool ReadBytes(unsigned char number, unsigned char *val){
  int rv = Wire.requestFrom(Address, number);
  if (rv == number)
  {
    for (unsigned char i = 0; i < number; i++)
    {
      val[i] = Wire.read();
    }
    return true;
  }
}

bool SHT31readTempHum(void) {
  unsigned char readbuffer[6];

  WriteCommand(SHT31_MEAS_HIGHREP);

  delay(20);

  //Wire.read(readbuffer, sizeof(readbuffer));
  ReadBytes(sizeof(readbuffer),readbuffer);

  if (readbuffer[2] != crc8(readbuffer, 2) ||
      readbuffer[5] != crc8(readbuffer + 3, 2))
    return false;

  int32_t stemp = (int32_t)(((uint32_t)readbuffer[0] << 8) | readbuffer[1]);
  // simplified (65536 instead of 65535) integer version of:
  // temp = (stemp * 175.0f) / 65535.0f - 45.0f;
  stemp = ((4375 * stemp) >> 14) - 4500;
  temp = (float)stemp / 100.0f;

  uint32_t shum = ((uint32_t)readbuffer[3] << 8) | readbuffer[4];
  // simplified (65536 instead of 65535) integer version of:
  // humidity = (shum * 100.0f) / 65535.0f;
  shum = (625 * shum) >> 12;
  humidity = (float)shum / 100.0f;

  return true;
}

float GetSHT31Temp(void){
    return temp;
}

float GetSHT31Humidity(void){
    return humidity;
}

unsigned char crc8(unsigned char  *data, int len) {
  const uint8_t POLYNOMIAL(0x31);
  uint8_t crc(0xFF);

  for (int j = len; j; --j) {
    crc ^= *data++;

    for (int i = 8; i; --i) {
      crc = (crc & 0x80) ? (crc << 1) ^ POLYNOMIAL : (crc << 1);
    }
  }
  return crc;
}