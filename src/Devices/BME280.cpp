#include "BME280.h"
#include <Arduino.h>
#include <Wire.h>

//Class BME280_SensorSettings.  This object is used to hold settings data.  The application
//uses this classes' data directly.  The settings are adopted and sent to the sensor
//at special times, such as .begin.  Some are used for doing math.
//
//This is a kind of bloated way to do this.  The trade-off is that the user doesn't
//need to deal with #defines or enums with bizarre names.
//
//A power user would strip out BME280_SensorSettings entirely, and send specific read and
//write command directly to the IC. (ST #defines below)

float BMETemp = 0;
float tempCorrection = 0;
float BMEhumidity = 0;
double _referencePressure = 0;
double BMEPresure = 0;
char I2CAddress = 0;
long t_fine;

struct BME280_SensorSettings
{
  public:
	
	//Main Interface and mode settings
    uint8_t commInterface;
    uint8_t I2CAddress;
    uint8_t chipSelectPin;

	//Deprecated settings
	uint8_t runMode;
	uint8_t tStandby;
	uint8_t filter;
	uint8_t tempOverSample;
	uint8_t pressOverSample;
	uint8_t humidOverSample;
    float tempCorrection; // correction of temperature - added to the result
};

//Used to hold the calibration constants.  These are used
//by the driver as measurements are being taking
struct SensorCalibration
{
  public:
	uint16_t dig_T1;
	int16_t dig_T2;
	int16_t dig_T3;
	
	uint16_t dig_P1;
	int16_t dig_P2;
	int16_t dig_P3;
	int16_t dig_P4;
	int16_t dig_P5;
	int16_t dig_P6;
	int16_t dig_P7;
	int16_t dig_P8;
	int16_t dig_P9;
	
	uint8_t dig_H1;
	int16_t dig_H2;
	uint8_t dig_H3;
	int16_t dig_H4;
	int16_t dig_H5;
	int8_t dig_H6;
	
};

SensorCalibration calibration;

#include <math.h>
#include "BME280.h"

uint8_t BME280begin()
{
	I2CAddress = 0x76;
	delay(2);  //Make sure sensor had enough time to turn on. BME280 requires 2ms to start up.

	//Check communication with IC before anything else
	uint8_t chipID = BME280readRegister(BME280_CHIP_ID_REG); //Should return 0x60 or 0x58
	Serial.println(chipID, HEX);
	if(chipID == 96){
        Serial.println("Chip ID OK");
    }
    else{
	    return(chipID); //This is not BMP nor BME!
    }

	//Reading all compensation data, range 0x88:A1, 0xE1:E7
	calibration.dig_T1 = ((uint16_t)((BME280readRegister(BME280_DIG_T1_MSB_REG) << 8) + BME280readRegister(BME280_DIG_T1_LSB_REG)));
	calibration.dig_T2 = ((int16_t)((BME280readRegister(BME280_DIG_T2_MSB_REG) << 8) + BME280readRegister(BME280_DIG_T2_LSB_REG)));
	calibration.dig_T3 = ((int16_t)((BME280readRegister(BME280_DIG_T3_MSB_REG) << 8) + BME280readRegister(BME280_DIG_T3_LSB_REG)));

	calibration.dig_P1 = ((uint16_t)((BME280readRegister(BME280_DIG_P1_MSB_REG) << 8) + BME280readRegister(BME280_DIG_P1_LSB_REG)));
	calibration.dig_P2 = ((int16_t)((BME280readRegister(BME280_DIG_P2_MSB_REG) << 8) + BME280readRegister(BME280_DIG_P2_LSB_REG)));
	calibration.dig_P3 = ((int16_t)((BME280readRegister(BME280_DIG_P3_MSB_REG) << 8) + BME280readRegister(BME280_DIG_P3_LSB_REG)));
	calibration.dig_P4 = ((int16_t)((BME280readRegister(BME280_DIG_P4_MSB_REG) << 8) + BME280readRegister(BME280_DIG_P4_LSB_REG)));
	calibration.dig_P5 = ((int16_t)((BME280readRegister(BME280_DIG_P5_MSB_REG) << 8) + BME280readRegister(BME280_DIG_P5_LSB_REG)));
	calibration.dig_P6 = ((int16_t)((BME280readRegister(BME280_DIG_P6_MSB_REG) << 8) + BME280readRegister(BME280_DIG_P6_LSB_REG)));
	calibration.dig_P7 = ((int16_t)((BME280readRegister(BME280_DIG_P7_MSB_REG) << 8) + BME280readRegister(BME280_DIG_P7_LSB_REG)));
	calibration.dig_P8 = ((int16_t)((BME280readRegister(BME280_DIG_P8_MSB_REG) << 8) + BME280readRegister(BME280_DIG_P8_LSB_REG)));
	calibration.dig_P9 = ((int16_t)((BME280readRegister(BME280_DIG_P9_MSB_REG) << 8) + BME280readRegister(BME280_DIG_P9_LSB_REG)));

	calibration.dig_H1 = ((uint8_t)(BME280readRegister(BME280_DIG_H1_REG)));
	calibration.dig_H2 = ((int16_t)((BME280readRegister(BME280_DIG_H2_MSB_REG) << 8) + BME280readRegister(BME280_DIG_H2_LSB_REG)));
	calibration.dig_H3 = ((uint8_t)(BME280readRegister(BME280_DIG_H3_REG)));
	calibration.dig_H4 = ((int16_t)((BME280readRegister(BME280_DIG_H4_MSB_REG) << 4) + (BME280readRegister(BME280_DIG_H4_LSB_REG) & 0x0F)));
	calibration.dig_H5 = ((int16_t)((BME280readRegister(BME280_DIG_H5_MSB_REG) << 4) + ((BME280readRegister(BME280_DIG_H4_LSB_REG) >> 4) & 0x0F)));
	calibration.dig_H6 = ((int8_t)BME280readRegister(BME280_DIG_H6_REG));

    Serial.println(calibration.dig_T1);
    Serial.println(calibration.dig_T2);
    Serial.println(calibration.dig_T3);

    Serial.println((uint16_t)((BME280readRegister(BME280_DIG_T1_MSB_REG) << 8) + BME280readRegister(BME280_DIG_T1_LSB_REG)));
    Serial.println((uint16_t)((BME280readRegister(BME280_DIG_P1_MSB_REG) << 8) + BME280readRegister(BME280_DIG_P1_LSB_REG)));
    Serial.println((uint16_t)((BME280readRegister(BME280_DIG_H2_MSB_REG) << 8) + BME280readRegister(BME280_DIG_H2_LSB_REG)));


	//Most of the time the sensor will be init with default values
	//But in case user has old/deprecated code, use the settings.x values
	BME280setStandbyTime(0);
	BME280setFilter(0);
	BME280setPressureOverSample(1); //Default of 1x oversample
	BME280setHumidityOverSample(1); //Default of 1x oversample
	BME280setTempOverSample(1); //Default of 1x oversample
	
	BME280setMode(MODE_NORMAL); //Go!
	
	return(BME280readRegister(BME280_CHIP_ID_REG)); //Should return 0x60
}

//Begin comm with BME280 over I2C
bool BME280beginI2C(){
	uint8_t chipID = BME280begin();

	if(chipID == 0x58) return(true); //Begin normal init with these settings. Should return chip ID of 0x58 for BMP
	if(chipID == 0x60) return(true); //Begin normal init with these settings. Should return chip ID of 0x60 for BME
	return(false);
}

//Set the mode bits in the ctrl_meas register
// Mode 00 = Sleep
// 01 and 10 = Forced
// 11 = Normal mode
void BME280setMode(unsigned char mode)
{
	if(mode > 0b11) mode = 0; //Error check. Default to sleep mode
	
	uint8_t controlData = BME280readRegister(BME280_CTRL_MEAS_REG);
	controlData &= ~( (1<<1) | (1<<0) ); //Clear the mode[1:0] bits
	controlData |= mode; //Set
	BME280writeRegister(BME280_CTRL_MEAS_REG, controlData);
}

//Gets the current mode bits in the ctrl_meas register
//Mode 00 = Sleep
// 01 and 10 = Forced
// 11 = Normal mode
uint8_t BME280getMode()
{
	uint8_t controlData = BME280readRegister(BME280_CTRL_MEAS_REG);
	return(controlData & 0b00000011); //Clear bits 7 through 2
}

//Set the standby bits in the config register
//tStandby can be:
//  0, 0.5ms
//  1, 62.5ms
//  2, 125ms
//  3, 250ms
//  4, 500ms
//  5, 1000ms
//  6, 10ms
//  7, 20ms
void BME280setStandbyTime(unsigned char timeSetting)
{
	if(timeSetting > 0b111) timeSetting = 0; //Error check. Default to 0.5ms
	
	uint8_t controlData = BME280readRegister(BME280_CONFIG_REG);
	controlData &= ~( (1<<7) | (1<<6) | (1<<5) ); //Clear the 7/6/5 bits
	controlData |= (timeSetting << 5); //Align with bits 7/6/5
	BME280writeRegister(BME280_CONFIG_REG, controlData);
}

//Set the filter bits in the config register
//filter can be off or number of FIR coefficients to use:
//  0, filter off
//  1, coefficients = 2
//  2, coefficients = 4
//  3, coefficients = 8
//  4, coefficients = 16
void BME280setFilter(unsigned char filterSetting)
{
	if(filterSetting > 0b111) filterSetting = 0; //Error check. Default to filter off
	
	uint8_t controlData = BME280readRegister(BME280_CONFIG_REG);
	controlData &= ~( (1<<4) | (1<<3) | (1<<2) ); //Clear the 4/3/2 bits
	controlData |= (filterSetting << 2); //Align with bits 4/3/2
	BME280writeRegister(BME280_CONFIG_REG, controlData);
}

//Set the temperature oversample value
//0 turns off temp sensing
//1 to 16 are valid over sampling values
void BME280setTempOverSample(unsigned char overSampleAmount)
{
	overSampleAmount = BME280checkSampleValue(overSampleAmount); //Error check
	
	uint8_t originalMode = BME280getMode(); //Get the current mode so we can go back to it at the end
	
	BME280setMode(MODE_SLEEP); //Config will only be writeable in sleep mode, so first go to sleep mode

	//Set the osrs_t bits (7, 6, 5) to overSampleAmount
	uint8_t controlData = BME280readRegister(BME280_CTRL_MEAS_REG);
	controlData &= ~( (1<<7) | (1<<6) | (1<<5) ); //Clear bits 765
	controlData |= overSampleAmount << 5; //Align overSampleAmount to bits 7/6/5
	BME280writeRegister(BME280_CTRL_MEAS_REG, controlData);
	
	BME280setMode(originalMode); //Return to the original user's choice
}

//Set the pressure oversample value
//0 turns off pressure sensing
//1 to 16 are valid over sampling values
void BME280setPressureOverSample(unsigned char overSampleAmount)
{
	overSampleAmount = BME280checkSampleValue(overSampleAmount); //Error check
	
	uint8_t originalMode = BME280getMode(); //Get the current mode so we can go back to it at the end
	
	BME280setMode(MODE_SLEEP); //Config will only be writeable in sleep mode, so first go to sleep mode

	//Set the osrs_p bits (4, 3, 2) to overSampleAmount
	uint8_t controlData = BME280readRegister(BME280_CTRL_MEAS_REG);
	controlData &= ~( (1<<4) | (1<<3) | (1<<2) ); //Clear bits 432
	controlData |= overSampleAmount << 2; //Align overSampleAmount to bits 4/3/2
	BME280writeRegister(BME280_CTRL_MEAS_REG, controlData);
	
	BME280setMode(originalMode); //Return to the original user's choice
}

//Set the humidity oversample value
//0 turns off humidity sensing
//1 to 16 are valid over sampling values
void BME280setHumidityOverSample(unsigned char overSampleAmount)
{
	overSampleAmount = BME280checkSampleValue(overSampleAmount); //Error check
	
	uint8_t originalMode = BME280getMode(); //Get the current mode so we can go back to it at the end
	
	BME280setMode(MODE_SLEEP); //Config will only be writeable in sleep mode, so first go to sleep mode

	//Set the osrs_h bits (2, 1, 0) to overSampleAmount
	uint8_t controlData = BME280readRegister(BME280_CTRL_HUMIDITY_REG);
	controlData &= ~( (1<<2) | (1<<1) | (1<<0) ); //Clear bits 2/1/0
	controlData |= overSampleAmount << 0; //Align overSampleAmount to bits 2/1/0
	BME280writeRegister(BME280_CTRL_HUMIDITY_REG, controlData);

	BME280setMode(originalMode); //Return to the original user's choice
}

//Validates an over sample value
//Allowed values are 0 to 16
//These are used in the humidty, pressure, and temp oversample functions
uint8_t BME280checkSampleValue(unsigned char userValue)
{
	switch(userValue) 
	{
		case(0): 
			return 0;
			break; //Valid
		case(1): 
			return 1;
			break; //Valid
		case(2): 
			return 2;
			break; //Valid
		case(4): 
			return 3;
			break; //Valid
		case(8): 
			return 4;
			break; //Valid
		case(16): 
			return 5;
			break; //Valid
		default: 
			return 1; //Default to 1x
			break; //Good
	}
}

//Set the global setting for the I2C address we want to communicate with
//Default is 0x77
void BME280setI2CAddress(unsigned char address)
{
    I2CAddress = address; //Set the I2C address for this device
}

//Check the measuring bit and return true while device is taking measurement
bool BME280isMeasuring(void)
{
	uint8_t stat = BME280readRegister(BME280_STAT_REG);
	return(stat & (1<<3)); //If the measuring bit (3) is set, return true
}

//Strictly resets.  Run .begin() afterwards
void BME280reset( void )
{
	BME280writeRegister(BME280_RST_REG, 0xB6);
	
}

//****************************************************************************//
//
//  Burst Measurement Section
//
//****************************************************************************//

//Read all sensor registers as a burst. See BME280 Datasheet section 4. Data readout
//tempScale = 0 for Celsius scale (default setting)
//tempScale = 1 for Fahrenheit scale
void BME280readAllMeasurements(unsigned char tempScale){
	
	uint8_t dataBurst[8];
	BME280readRegisterRegion(dataBurst, BME280_MEASUREMENTS_REG, 8);
	
	if(tempScale == 0){
		BME280readTempCFromBurst(dataBurst);
	}else{
		BME280readTempFFromBurst(dataBurst);
	}
	BME280readFloatPressureFromBurst(dataBurst);
	BME280readFloatHumidityFromBurst(dataBurst);
}

//****************************************************************************//
//
//  Pressure Section
//
//****************************************************************************//
float BME280readFloatPressure( void )
{

	// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
	// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
    uint8_t buffer[3];
	BME280readRegisterRegion(buffer, BME280_PRESSURE_MSB_REG, 3);
    int32_t adc_P = ((uint32_t)buffer[0] << 12) | ((uint32_t)buffer[1] << 4) | ((buffer[2] >> 4) & 0x0F);
	
	int64_t var1, var2, p_acc;
	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)calibration.dig_P6;
	var2 = var2 + ((var1 * (int64_t)calibration.dig_P5)<<17);
	var2 = var2 + (((int64_t)calibration.dig_P4)<<35);
	var1 = ((var1 * var1 * (int64_t)calibration.dig_P3)>>8) + ((var1 * (int64_t)calibration.dig_P2)<<12);
	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)calibration.dig_P1)>>33;
	if (var1 == 0)
	{
		return 0; // avoid exception caused by division by zero
	}
	p_acc = 1048576 - adc_P;
	p_acc = (((p_acc<<31) - var2)*3125)/var1;
	var1 = (((int64_t)calibration.dig_P9) * (p_acc>>13) * (p_acc>>13)) >> 25;
	var2 = (((int64_t)calibration.dig_P8) * p_acc) >> 19;
	p_acc = ((p_acc + var1 + var2) >> 8) + (((int64_t)calibration.dig_P7)<<4);
	
	return (float)p_acc / 25600.0;
	
}

void BME280readFloatPressureFromBurst(unsigned char buffer[]){

	// Set pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
	// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
  
  int32_t adc_P = ((uint32_t)buffer[0] << 12) | ((uint32_t)buffer[1] << 4) | ((buffer[2] >> 4) & 0x0F);
	
	int64_t var1, var2, p_acc;
	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)calibration.dig_P6;
	var2 = var2 + ((var1 * (int64_t)calibration.dig_P5)<<17);
	var2 = var2 + (((int64_t)calibration.dig_P4)<<35);
	var1 = ((var1 * var1 * (int64_t)calibration.dig_P3)>>8) + ((var1 * (int64_t)calibration.dig_P2)<<12);
	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)calibration.dig_P1)>>33;
	if (var1 == 0)
	{
		BMEPresure = 0; // avoid exception caused by division by zero
	}
	else
	{
		p_acc = 1048576 - adc_P;
		p_acc = (((p_acc<<31) - var2)*3125)/var1;
		var1 = (((int64_t)calibration.dig_P9) * (p_acc>>13) * (p_acc>>13)) >> 25;
		var2 = (((int64_t)calibration.dig_P8) * p_acc) >> 19;
		p_acc = ((p_acc + var1 + var2) >> 8) + (((int64_t)calibration.dig_P7)<<4);
		
		BMEPresure = (float)p_acc / 256.0;
	}
}

// Sets the internal variable _referencePressure so the altitude is calculated properly.
// This is also known as "sea level pressure" and is in Pascals. The value is probably
// within 10% of 101325. This varies based on the weather:
// https://en.wikipedia.org/wiki/Atmospheric_pressure#Mean_sea-level_pressure
//
// if you are concerned about accuracy or precision, make sure to pull the
// "sea level pressure"value from a trusted source like NOAA.
void BME280setReferencePressure(float refPressure){
	_referencePressure = refPressure;
}

//Return the local reference pressure
float BME280getReferencePressure(){
	return(_referencePressure);
}

float BME280readFloatAltitudeMeters( void ){
	float heightOutput = 0;
	
  // Getting height from a pressure reading is called the "international barometric height formula".
  // The magic value of 44330.77 was adjusted in issue #30.
  // There's also some discussion of it here: https://www.sparkfun.com/tutorials/253
  // This calculation is NOT designed to work on non-Earthlike planets such as Mars or Venus;
  // see NRLMSISE-00. That's why it is the "international" formula, not "interplanetary".
  // Sparkfun is not liable for incorrect altitude calculations from this
  // code on those planets. Interplanetary selfies are welcome, however.
	heightOutput = ((float)-44330.77)*(pow(((float)BME280readFloatPressure()/(float)_referencePressure), 0.190263) - (float)1); //Corrected, see issue 30
	return heightOutput;
}

float BME280readFloatAltitudeFeet( void ){
	float heightOutput = 0;
	
	heightOutput = BME280readFloatAltitudeMeters() * 3.28084;
	return heightOutput;
	
}

//****************************************************************************//
//
//  Humidity Section
//
//****************************************************************************//
float BME280readFloatHumidity( void ){
	
	// Returns humidity in %RH as unsigned 32 bit integer in Q22. 10 format (22 integer and 10 fractional bits).
	// Output value of “47445” represents 47445/1024 = 46. 333 %RH
    uint8_t buffer[2];
	BME280readRegisterRegion(buffer, BME280_HUMIDITY_MSB_REG, 2);
    int32_t adc_H = ((uint32_t)buffer[0] << 8) | ((uint32_t)buffer[1]);
	int32_t var1;
	var1 = (t_fine - ((int32_t)76800));
	var1 = (((((adc_H << 14) - (((int32_t)calibration.dig_H4) << 20) - (((int32_t)calibration.dig_H5) * var1)) +
	((int32_t)16384)) >> 15) * (((((((var1 * ((int32_t)calibration.dig_H6)) >> 10) * (((var1 * ((int32_t)calibration.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
	((int32_t)calibration.dig_H2) + 8192) >> 14));
	var1 = (var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * ((int32_t)calibration.dig_H1)) >> 4));
	var1 = (var1 < 0 ? 0 : var1);
	var1 = (var1 > 419430400 ? 419430400 : var1);
    
	return (float)(var1>>12) / 1024.0;
}

void BME280readFloatHumidityFromBurst(unsigned char buffer[])
{
	
	// Set humidity in %RH as unsigned 32 bit integer in Q22. 10 format (22 integer and 10 fractional bits).
	// Output value of “47445” represents 47445/1024 = 46. 333 %RH
  int32_t adc_H = ((uint32_t)buffer[6] << 8) | ((uint32_t)buffer[7]);
	
	int32_t var1;
	var1 = (t_fine - ((int32_t)76800));
	var1 = (((((adc_H << 14) - (((int32_t)calibration.dig_H4) << 20) - (((int32_t)calibration.dig_H5) * var1)) +
	((int32_t)16384)) >> 15) * (((((((var1 * ((int32_t)calibration.dig_H6)) >> 10) * (((var1 * ((int32_t)calibration.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
	((int32_t)calibration.dig_H2) + 8192) >> 14));
	var1 = (var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * ((int32_t)calibration.dig_H1)) >> 4));
	var1 = (var1 < 0 ? 0 : var1);
	var1 = (var1 > 419430400 ? 419430400 : var1);

	BMEhumidity = (float)(var1>>12) / 1024.0;
}

//****************************************************************************//
//
//  Temperature Section
//
//****************************************************************************//

void BME280setTemperatureCorrection(float corr){
	tempCorrection = corr;
}

float BME280readTempC( void ){
	// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
	// t_fine carries fine temperature as global value

	//get the reading (adc_T);
    uint8_t buffer[3];
	BME280readRegisterRegion(buffer, BME280_TEMPERATURE_MSB_REG, 3);
    int32_t adc_T = ((uint32_t)buffer[0] << 12) | ((uint32_t)buffer[1] << 4) | ((buffer[2] >> 4) & 0x0F);

	//By datasheet, calibrate
	int64_t var1, var2;

	var1 = ((((adc_T>>3) - ((int32_t)calibration.dig_T1<<1))) * ((int32_t)calibration.dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((int32_t)calibration.dig_T1)) * ((adc_T>>4) - ((int32_t)calibration.dig_T1))) >> 12) *
	((int32_t)calibration.dig_T3)) >> 14;
	t_fine = var1 + var2;
	t_fine = var1 + var2;
	float output = (t_fine * 5 + 128) >> 8;

	output = output / 100 + tempCorrection;
	
	return output;
}

float BME280readTempFromBurst(unsigned char buffer[]){
  int32_t adc_T = ((uint32_t)buffer[3] << 12) | ((uint32_t)buffer[4] << 4) | ((buffer[5] >> 4) & 0x0F);

	//By datasheet, calibrate
	int64_t var1, var2;

	var1 = ((((adc_T>>3) - ((int32_t)calibration.dig_T1<<1))) * ((int32_t)calibration.dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((int32_t)calibration.dig_T1)) * ((adc_T>>4) - ((int32_t)calibration.dig_T1))) >> 12) *
	((int32_t)calibration.dig_T3)) >> 14;
	t_fine = var1 + var2;
	float output = (t_fine * 5 + 128) >> 8;

	output = output / 100 + tempCorrection;
	
 	return output;
}

void BME280readTempCFromBurst(unsigned char buffer[]){
  BMETemp = BME280readTempFromBurst(buffer);
}

float BME280readTempF( void ){
	float output = BME280readTempC();
	output = (output * 9) / 5 + 32;

	return output;
}

void BME280readTempFFromBurst(unsigned char buffer[]){
  float output = BME280readTempFromBurst(buffer);
	output = (output * 9) / 5 + 32;

    BMETemp = output;
}

//****************************************************************************//
//
//  Utility
//
//****************************************************************************//
void BME280readRegisterRegion(unsigned char *outputPointer , unsigned char offset, unsigned char length){
	//define pointer that will point to the external space
	uint8_t i = 0;
	char c = 0;

	Wire.beginTransmission(I2CAddress);
	Wire.write(offset);
	Wire.endTransmission();

	// request bytes from slave device
	Wire.requestFrom((int)I2CAddress, (int)length);
	while ( (Wire.available()) && (i < length)){
		c = Wire.read(); // receive a byte as character
		*outputPointer = c;
		outputPointer++;
		i++;
	}
}

unsigned char BME280readRegister(unsigned char offset){
	//Return value
	unsigned char result = 0;
	Wire.beginTransmission(I2CAddress);
	Wire.write(offset);
	Wire.endTransmission();

	Wire.requestFrom((int)I2CAddress, 1);
	result = Wire.read(); // receive a byte as a proper uint8_t
    return result;
}

unsigned int BME280readRegisterInt16( unsigned char offset ){
	unsigned char myBuffer[2];
	BME280readRegisterRegion(myBuffer, offset, 2);  //Does memory transfer
	unsigned int output = (unsigned int)myBuffer[0] | (unsigned int)(myBuffer[1] << 8);
	
	return output;
}

void BME280writeRegister(unsigned char offset, unsigned char dataToWrite){
	Wire.beginTransmission(I2CAddress);
	Wire.write(offset);
	Wire.write(dataToWrite);
	Wire.endTransmission();
}

