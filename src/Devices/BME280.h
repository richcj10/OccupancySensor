#ifndef BME280_H
#define BME280_H

#define MODE_SLEEP 0b00
#define MODE_FORCED 0b01
#define MODE_NORMAL 0b11

//Register names:
#define BME280_DIG_T1_LSB_REG			0x88
#define BME280_DIG_T1_MSB_REG			0x89
#define BME280_DIG_T2_LSB_REG			0x8A
#define BME280_DIG_T2_MSB_REG			0x8B
#define BME280_DIG_T3_LSB_REG			0x8C
#define BME280_DIG_T3_MSB_REG			0x8D
#define BME280_DIG_P1_LSB_REG			0x8E
#define BME280_DIG_P1_MSB_REG			0x8F
#define BME280_DIG_P2_LSB_REG			0x90
#define BME280_DIG_P2_MSB_REG			0x91
#define BME280_DIG_P3_LSB_REG			0x92
#define BME280_DIG_P3_MSB_REG			0x93
#define BME280_DIG_P4_LSB_REG			0x94
#define BME280_DIG_P4_MSB_REG			0x95
#define BME280_DIG_P5_LSB_REG			0x96
#define BME280_DIG_P5_MSB_REG			0x97
#define BME280_DIG_P6_LSB_REG			0x98
#define BME280_DIG_P6_MSB_REG			0x99
#define BME280_DIG_P7_LSB_REG			0x9A
#define BME280_DIG_P7_MSB_REG			0x9B
#define BME280_DIG_P8_LSB_REG			0x9C
#define BME280_DIG_P8_MSB_REG			0x9D
#define BME280_DIG_P9_LSB_REG			0x9E
#define BME280_DIG_P9_MSB_REG			0x9F
#define BME280_DIG_H1_REG				0xA1
#define BME280_CHIP_ID_REG				0xD0 //Chip ID
#define BME280_RST_REG					0xE0 //Softreset Reg
#define BME280_DIG_H2_LSB_REG			0xE1
#define BME280_DIG_H2_MSB_REG			0xE2
#define BME280_DIG_H3_REG				0xE3
#define BME280_DIG_H4_MSB_REG			0xE4
#define BME280_DIG_H4_LSB_REG			0xE5
#define BME280_DIG_H5_MSB_REG			0xE6
#define BME280_DIG_H6_REG				0xE7
#define BME280_CTRL_HUMIDITY_REG		0xF2 //Ctrl Humidity Reg
#define BME280_STAT_REG					0xF3 //Status Reg
#define BME280_CTRL_MEAS_REG			0xF4 //Ctrl Measure Reg
#define BME280_CONFIG_REG				0xF5 //Configuration Reg
#define BME280_MEASUREMENTS_REG			0xF7 //Measurements register start
#define BME280_PRESSURE_MSB_REG			0xF7 //Pressure MSB
#define BME280_PRESSURE_LSB_REG			0xF8 //Pressure LSB
#define BME280_PRESSURE_XLSB_REG		0xF9 //Pressure XLSB
#define BME280_TEMPERATURE_MSB_REG		0xFA //Temperature MSB
#define BME280_TEMPERATURE_LSB_REG		0xFB //Temperature LSB
#define BME280_TEMPERATURE_XLSB_REG		0xFC //Temperature XLSB
#define BME280_HUMIDITY_MSB_REG			0xFD //Humidity MSB
#define BME280_HUMIDITY_LSB_REG			0xFE //Humidity LSB


unsigned char BME280begin();
//Begin comm with BME280 over I2C
bool BME280beginI2C();
void BME280setMode(unsigned char mode);
unsigned char BME280getMode();
void BME280setStandbyTime(unsigned char timeSetting);
void BME280setFilter(unsigned char filterSetting);
void BME280setTempOverSample(unsigned char overSampleAmount);
void BME280setPressureOverSample(unsigned char overSampleAmount);
void BME280setHumidityOverSample(unsigned char overSampleAmount);
unsigned char BME280checkSampleValue(unsigned char userValue);
void BME280setI2CAddress(unsigned char address);
bool BME280isMeasuring(void);
void BME280reset( void );
void BME280readAllMeasurements( unsigned char tempScale);
float BME280readFloatPressure( void );
void BME280readFloatPressureFromBurst(unsigned char buffer[]);
void BME280setReferencePressure(float refPressure);
float BME280getReferencePressure();
float BME280readFloatAltitudeMeters( void );
float BME280readFloatAltitudeFeet( void );
float BME280readFloatHumidity( void );
void BME280readFloatHumidityFromBurst(unsigned char buffer[]);
void BME280setTemperatureCorrection(float corr);
float BME280readTempC( void );
float BME280readTempFromBurst(unsigned char buffer[]);
void BME280readTempCFromBurst(unsigned char buffer[]);
float BME280readTempF( void );
void BME280readTempFFromBurst(unsigned char buffer[]);
void BME280readRegisterRegion(unsigned char *outputPointer , unsigned char offset, unsigned char length);
unsigned char BME280readRegister(unsigned char offset);
unsigned int BME280readRegisterInt16( unsigned char offset );
void BME280writeRegister(unsigned char offset, unsigned char dataToWrite);

#endif