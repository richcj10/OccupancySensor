#ifndef TSL2591_H
#define TSL2591_H

#define TSL2591_VISIBLE (2)      ///< (channel 0) - (channel 1)
#define TSL2591_INFRARED (1)     ///< channel 1
#define TSL2591_FULLSPECTRUM (0) ///< channel 0

#define TSL2591_ADDR (0x29) ///< Default I2C address

#define TSL2591_COMMAND_BIT                                                    \
  (0xA0) ///< 1010 0000: bits 7 and 5 for 'command normal'

///! Special Function Command for "Clear ALS and no persist ALS interrupt"
#define TSL2591_CLEAR_INT (0xE7)
///! Special Function Command for "Interrupt set - forces an interrupt"
#define TSL2591_TEST_INT (0xE4)

#define TSL2591_WORD_BIT (0x20)  ///< 1 = read/write word (rather than byte)
#define TSL2591_BLOCK_BIT (0x10) ///< 1 = using block read/write

#define TSL2591_ENABLE_POWEROFF (0x00) ///< Flag for ENABLE register to disable
#define TSL2591_ENABLE_POWERON (0x01)  ///< Flag for ENABLE register to enable
#define TSL2591_ENABLE_AEN                                                     \
  (0x02) ///< ALS Enable. This field activates ALS function. Writing a one
         ///< activates the ALS. Writing a zero disables the ALS.
#define TSL2591_ENABLE_AIEN                                                    \
  (0x10) ///< ALS Interrupt Enable. When asserted permits ALS interrupts to be
         ///< generated, subject to the persist filter.
#define TSL2591_ENABLE_NPIEN                                                   \
  (0x80) ///< No Persist Interrupt Enable. When asserted NP Threshold conditions
         ///< will generate an interrupt, bypassing the persist filter

#define TSL2591_LUX_DF (408.0F)   ///< Lux cooefficient
#define TSL2591_LUX_COEFB (1.64F) ///< CH0 coefficient
#define TSL2591_LUX_COEFC (0.59F) ///< CH1 coefficient A
#define TSL2591_LUX_COEFD (0.86F) ///< CH2 coefficient B

/// TSL2591 Register map
enum {
  TSL2591_REGISTER_ENABLE = 0x00,          // Enable register
  TSL2591_REGISTER_CONTROL = 0x01,         // Control register
  TSL2591_REGISTER_THRESHOLD_AILTL = 0x04, // ALS low threshold lower byte
  TSL2591_REGISTER_THRESHOLD_AILTH = 0x05, // ALS low threshold upper byte
  TSL2591_REGISTER_THRESHOLD_AIHTL = 0x06, // ALS high threshold lower byte
  TSL2591_REGISTER_THRESHOLD_AIHTH = 0x07, // ALS high threshold upper byte
  TSL2591_REGISTER_THRESHOLD_NPAILTL =
      0x08, // No Persist ALS low threshold lower byte
  TSL2591_REGISTER_THRESHOLD_NPAILTH =
      0x09, // No Persist ALS low threshold higher byte
  TSL2591_REGISTER_THRESHOLD_NPAIHTL =
      0x0A, // No Persist ALS high threshold lower byte
  TSL2591_REGISTER_THRESHOLD_NPAIHTH =
      0x0B, // No Persist ALS high threshold higher byte
  TSL2591_REGISTER_PERSIST_FILTER = 0x0C, // Interrupt persistence filter
  TSL2591_REGISTER_PACKAGE_PID = 0x11,    // Package Identification
  TSL2591_REGISTER_DEVICE_ID = 0x12,      // Device Identification
  TSL2591_REGISTER_DEVICE_STATUS = 0x13,  // Internal Status
  TSL2591_REGISTER_CHAN0_LOW = 0x14,      // Channel 0 data, low byte
  TSL2591_REGISTER_CHAN0_HIGH = 0x15,     // Channel 0 data, high byte
  TSL2591_REGISTER_CHAN1_LOW = 0x16,      // Channel 1 data, low byte
  TSL2591_REGISTER_CHAN1_HIGH = 0x17,     // Channel 1 data, high byte
};


#define TSL2591_INTEGRATIONTIME_100MS 0x00
#define TSL2591_INTEGRATIONTIME_200MS 0x01
#define TSL2591_INTEGRATIONTIME_300MS 0x02
#define TSL2591_INTEGRATIONTIME_400MS 0x03
#define TSL2591_INTEGRATIONTIME_500MS 0x04
#define TSL2591_INTEGRATIONTIME_600MS 0x05

#define TSL2591_GAIN_LOW 0x00  // low gain (1x)
#define TSL2591_GAIN_MED 0x10 // medium gain (25x)
#define TSL2591_GAIN_HIGH 0x20 // medium gain (428x)
#define TSL2591_GAIN_MAX 0x30  // max gain (9876x)


#include <Arduino.h>

boolean TSL2591begin();
void TSL2591enable(void);
void TSL2591disable(void);
void TSL2591setGain(char gain);
void TSL2591setTiming(char integration);
float TSL2591calculateLux(uint16_t ch0, uint16_t ch1);
uint32_t TSL2591getFullLuminosity(void);
uint16_t TSL2591getLuminosity(uint8_t channel);
void TSL2591clearInterrupt();
uint8_t TSL2591getStatus(void);
uint8_t TSL2591read8(uint8_t reg);
uint16_t TSL2591read16(uint8_t reg);
void TSL2591write8(uint8_t reg, uint8_t value);
void TSL2591write8(uint8_t reg);
#endif