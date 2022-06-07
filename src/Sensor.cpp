#include "Sensor.h"
#include "Define.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <Devices\SHT31.h>
#include <Devices\BME280.h>
#include <Devices\TSL2591.h>

#include "Define.h"
#define PIN        5 // On Trinket or Gemma, suggest changing this to 1

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS 1 // Popular NeoPixel ring 

float Temp1 = 0;
float Temp2 = 0;
float Humidity1 = 0;
float Humidity2 = 0;
int hPA = 0;
int lux = 0;
float Vin = 0;

// When setting up the NeoPixel library, we tell it how many pixels,
// and which pin to use to send signals. Note that for older NeoPixel
// strips you might need to change the third parameter -- see the
// strandtest example for more information on possible values.
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

void SensorStart(){
  pixels.begin();
  pixels.clear();

  Wire.begin();
  SHT31Start(0x44);
  BME280beginI2C();
  if(TSL2591begin()){
    //Serial.println("Lux OK");
  }
}

void ReadSensors(){
  SHT31readTempHum();
  float Temp1 = GetSHT31Temp();
  float Temp2 = BME280readTempF();
  float Humidity1 = GetSHT31Humidity();
  float Humidity2 = BME280readFloatHumidity();
  int hPA = BME280readFloatPressure();
  int lux = TSL2591getLuminosity(TSL2591_VISIBLE);
  SampleInput();
}

void UpdatePixels(unsigned char R, unsigned char G, unsigned char B){
    pixels.setPixelColor(0, pixels.Color(R, G, B));
    pixels.show();
}

int GetSensorValues(char Type){
  switch (Type)
  {
  case SENSOR_INPUT_TEMP1:
    return Temp1;
    break;
  case SENSOR_INPUT_TEMP2:
    return Temp2;
    break;
  case SENSOR_INPUT_HUMID1:
    return Humidity1;
    break;
  case SENSOR_INPUT_HUMID2:
    return Humidity2;
    break;
  case SENSOR_INPUT_PRES:
    return hPA;
    break;  
  case SENSOR_INPUT_LUX:
    return lux;
    break;  
  default:
    return 0;
    break;
  }
  return 0;
}

#define numReadings 10

int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
long total = 0;                  // the running total

void SampleInput(){
  total = total - readings[readIndex];
  // read from the sensor:
  readings[readIndex] = analogRead(VOLTSENSE);
  // add the reading to the total:
  total = total + readings[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }

  // calculate the average:
  Vin = (total / numReadings)*(3.3/1024)*5.57;
  Serial.print("Vin = ");
  Serial.println(Vin);
}