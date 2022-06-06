#include "TSL2591.h"
#include <stdlib.h>
#include <Arduino.h>

#include <Wire.h>

char intigration = TSL2591_INTEGRATIONTIME_100MS;
char gain = TSL2591_GAIN_MED;

/**************************************************************************/
/*!
    @brief   Setups the I2C interface and hardware, identifies if chip is found
    @param   theWire a reference to TwoWire instance
    @param   addr The I2C adress of the sensor (Default 0x29)
    @returns True if a TSL2591 is found, false on any failure
*/
/**************************************************************************/
boolean TSL2591begin() {
  uint8_t id = TSL2591read8(TSL2591_COMMAND_BIT | TSL2591_REGISTER_DEVICE_ID);
  if (id != 0x50) {
    return false;
  }
  TSL2591enable();
  // Serial.println("Found Adafruit_TSL2591");
  // Set default integration time and gain
  TSL2591setTiming(intigration);
  TSL2591setGain(gain);

  // Note: by default, the device is in power down mode on bootup
  //TSL2591disable();

  return true;
}

/**************************************************************************/
/*!
    @brief  Enables the chip, so it's ready to take readings
*/
/**************************************************************************/
void TSL2591enable(void) {
  // Enable the device by setting the control bit to 0x01
  TSL2591write8(TSL2591_COMMAND_BIT | TSL2591_REGISTER_ENABLE,
         TSL2591_ENABLE_POWERON | TSL2591_ENABLE_AEN | TSL2591_ENABLE_AIEN |
             TSL2591_ENABLE_NPIEN);
}

/**************************************************************************/
/*!
    @brief Disables the chip, so it's in power down mode
*/
/**************************************************************************/
void TSL2591disable(void) {
  // Disable the device by setting the control bit to 0x00
  TSL2591write8(TSL2591_COMMAND_BIT | TSL2591_REGISTER_ENABLE,
         TSL2591_ENABLE_POWEROFF);
}

/************************************************************************/
/*!
    @brief  Setter for sensor light gain
    @param  gain {@link tsl2591Gain_t} gain value
*/
/**************************************************************************/
void TSL2591setGain(char g) {
  gain = g;
  TSL2591write8(TSL2591_COMMAND_BIT | TSL2591_REGISTER_CONTROL, intigration | gain);
}

/************************************************************************/
/*!
    @brief  Setter for sensor integration time setting
    @param integration {@link tsl2591IntegrationTime_t} integration time setting
*/
/**************************************************************************/
void TSL2591setTiming(char i) {
    intigration = i;
    TSL2591write8(TSL2591_COMMAND_BIT | TSL2591_REGISTER_CONTROL, intigration | gain);
}

/************************************************************************/
/*!
    @brief  Calculates the visible Lux based on the two light sensors
    @param  ch0 Data from channel 0 (IR+Visible)
    @param  ch1 Data from channel 1 (IR)
    @returns Lux, based on AMS coefficients (or < 0 if overflow)
*/
/**************************************************************************/
float TSL2591calculateLux(uint16_t ch0, uint16_t ch1) {
  float atime, again;
  float cpl, lux1, lux2, lux;
  uint32_t chan0, chan1;

  // Check for overflow conditions first
  if ((ch0 == 0xFFFF) | (ch1 == 0xFFFF)) {
    // Signal an overflow
    return -1;
  }

  // Note: This algorithm is based on preliminary coefficients
  // provided by AMS and may need to be updated in the future

  switch (intigration) {
  case TSL2591_INTEGRATIONTIME_100MS:
    atime = 100.0F;
    break;
  case TSL2591_INTEGRATIONTIME_200MS:
    atime = 200.0F;
    break;
  case TSL2591_INTEGRATIONTIME_300MS:
    atime = 300.0F;
    break;
  case TSL2591_INTEGRATIONTIME_400MS:
    atime = 400.0F;
    break;
  case TSL2591_INTEGRATIONTIME_500MS:
    atime = 500.0F;
    break;
  case TSL2591_INTEGRATIONTIME_600MS:
    atime = 600.0F;
    break;
  default: // 100ms
    atime = 100.0F;
    break;
  }

  switch (gain) {
  case TSL2591_GAIN_LOW:
    again = 1.0F;
    break;
  case TSL2591_GAIN_MED:
    again = 25.0F;
    break;
  case TSL2591_GAIN_HIGH:
    again = 428.0F;
    break;
  case TSL2591_GAIN_MAX:
    again = 9876.0F;
    break;
  default:
    again = 1.0F;
    break;
  }

  // cpl = (ATIME * AGAIN) / DF
  cpl = (atime * again) / TSL2591_LUX_DF;

  // Original lux calculation (for reference sake)
  // lux1 = ( (float)ch0 - (TSL2591_LUX_COEFB * (float)ch1) ) / cpl;
  // lux2 = ( ( TSL2591_LUX_COEFC * (float)ch0 ) - ( TSL2591_LUX_COEFD *
  // (float)ch1 ) ) / cpl; lux = lux1 > lux2 ? lux1 : lux2;

  // Alternate lux calculation 1
  // See: https://github.com/adafruit/Adafruit_TSL2591_Library/issues/14
  lux = (((float)ch0 - (float)ch1)) * (1.0F - ((float)ch1 / (float)ch0)) / cpl;

  // Alternate lux calculation 2
  // lux = ( (float)ch0 - ( 1.7F * (float)ch1 ) ) / cpl;

  // Signal I2C had no errors
  return lux;
}

/************************************************************************/
/*!
    @brief  Reads the raw data from both light channels
    @returns 32-bit raw count where high word is IR, low word is IR+Visible
*/
/**************************************************************************/
uint32_t TSL2591getFullLuminosity(void) {

  // Wait x ms for ADC to complete
  for (uint8_t d = 0; d <= intigration; d++) {
    delay(120);
  }

  // CHAN0 must be read before CHAN1
  // See: https://forums.adafruit.com/viewtopic.php?f=19&t=124176
  uint32_t x;
  uint16_t y;
  y = TSL2591read16(TSL2591_COMMAND_BIT | TSL2591_REGISTER_CHAN0_LOW);
  x = TSL2591read16(TSL2591_COMMAND_BIT | TSL2591_REGISTER_CHAN1_LOW);
  x <<= 16;
  x |= y;

  return x;
}

/************************************************************************/
/*!
    @brief  Reads the raw data from the channel
    @param  channel Can be 0 (IR+Visible, 1 (IR) or 2 (Visible only)
    @returns 16-bit raw count, or 0 if channel is invalid
*/
/**************************************************************************/
uint16_t TSL2591getLuminosity(uint8_t channel) {
  uint32_t x = TSL2591getFullLuminosity();
  if (channel == TSL2591_FULLSPECTRUM) {
    // Reads two byte value from channel 0 (visible + infrared)
    return (x & 0xFFFF);
  } else if (channel == TSL2591_INFRARED) {
    // Reads two byte value from channel 1 (infrared)
    return (x >> 16);
  } else if (channel == TSL2591_VISIBLE) {
    // Reads all and subtracts out just the visible!
    return ((x & 0xFFFF) - (x >> 16));
  }

  // unknown channel!
  return 0;
}

/************************************************************************/
/*!
    @brief  Gets the most recent sensor event from the hardware status register.
    @return Sensor status as a byte. Bit 0 is ALS Valid. Bit 4 is ALS Interrupt.
   Bit 5 is No-persist Interrupt.
*/
/**************************************************************************/
uint8_t TSL2591getStatus(void) {
  uint8_t x;
  x = TSL2591read8(TSL2591_COMMAND_BIT | TSL2591_REGISTER_DEVICE_STATUS);
  return x;
}

uint8_t TSL2591read8(uint8_t reg) {
  uint8_t buffer[1];
  //buffer[0] = reg;
  Wire.beginTransmission(TSL2591_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(TSL2591_ADDR, 1);
  buffer[0] = Wire.read();
  return buffer[0];
}

uint16_t TSL2591read16(uint8_t reg) {
  uint8_t buffer[2];
  buffer[0] = reg;
  Wire.beginTransmission(TSL2591_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(TSL2591_ADDR, 2);
  buffer[0] = Wire.read();
  buffer[1] = Wire.read();
  return uint16_t(buffer[1]) << 8 | uint16_t(buffer[0]);
}

void TSL2591write8(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(TSL2591_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void TSL2591write8(uint8_t reg) {
  Wire.beginTransmission(TSL2591_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
}