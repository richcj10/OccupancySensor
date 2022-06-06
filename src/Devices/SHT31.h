#ifndef SHT31_H
#define SHT31_H

#define SHT31_DEFAULT_ADDR 0x44 /**< SHT31 Default Address */
#define SHT31_MEAS_HIGHREP_STRETCH 0x2C06 /**< Measurement High Repeatability with Clock Stretch Enabled */
#define SHT31_MEAS_MEDREP_STRETCH 0x2C0D /**< Measurement Medium Repeatability with Clock Stretch Enabled */
#define SHT31_MEAS_LOWREP_STRETCH 0x2C10 /**< Measurement Low Repeatability with Clock Stretch Enabled*/
#define SHT31_MEAS_HIGHREP 0x2400 /**< Measurement High Repeatability with Clock Stretch Disabled */
#define SHT31_MEAS_MEDREP 0x240B /**< Measurement Medium Repeatability with Clock Stretch Disabled */
#define SHT31_MEAS_LOWREP 0x2416 /**< Measurement Low Repeatability with Clock Stretch Disabled */
#define SHT31_READSTATUS 0xF32D   /**< Read Out of Status Register */
#define SHT31_CLEARSTATUS 0x3041  /**< Clear Status */
#define SHT31_SOFTRESET 0x30A2    /**< Soft Reset */
#define SHT31_HEATEREN 0x306D     /**< Heater Enable */
#define SHT31_HEATERDIS 0x3066    /**< Heater Disable */
#define SHT31_REG_HEATER_BIT 0x0d /**< Status Register Heater Bit */

float GetSHT31Temp(void);
float GetSHT31Humidity(void);

void SHT31Start(char A);
void SHT31Reset(void);
void SHT31heater(bool h);
bool SHT31HeaterEnabled();
bool WriteCommand(unsigned int command);
unsigned int ReadStatus(void);
bool SHT31readTempHum(void);
bool ReadBytes(unsigned char n, unsigned char *val);
unsigned char crc8(unsigned char *data, int len);

#endif