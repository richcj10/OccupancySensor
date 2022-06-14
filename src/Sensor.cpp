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
int VinAdc[VOLT_AVG];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
long Vintotal = 0;                  // the running total
bool lastMotionState = 0;
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50; 
bool MotionState = 0;

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
  for(char i = 0;i<10;i++){
    SampleVin();
  }
}

void ReadSensors(){
  SHT31readTempHum();
  Temp1 = GetSHT31Temp();
  Temp2 = BME280readTempF();
  Humidity1 = GetSHT31Humidity();
  Humidity2 = BME280readFloatHumidity();
  hPA = BME280readFloatPressure();
  lux = TSL2591getLuminosity(TSL2591_VISIBLE);
  SampleVin();
  DebounceMotionSensor();
}

void UpdatePixels(unsigned char R, unsigned char G, unsigned char B){
    pixels.setPixelColor(0, pixels.Color(R, G, B));
    pixels.show();
}

float GetSensorValues(char Type){
  switch (Type){
    case SENSOR_INPUT_VIN:
      return Vin;
      break;
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
    case SENSOR_INPUT_MOTION:
      return MotionState;
      break;  
    default:
      return 0;
      break;
  }
  return 0;
}

void SampleVin(){
  Vintotal = Vintotal - VinAdc[readIndex];
  // read from the sensor:
  VinAdc[readIndex] = analogRead(VOLTSENSE);
  // add the reading to the total:
  Vintotal = Vintotal + VinAdc[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= VOLT_AVG) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }

  // calculate the average:
  Vin = (Vintotal / VOLT_AVG)*(3.3/1024)*5.0;
  //Serial.print("Vin = ");
  //Serial.println(Vin);
}

void DebounceMotionSensor(){
  bool reading = digitalRead(MOTION);

  if (reading != lastMotionState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > 250) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != MotionState) {
      MotionState = reading;
    }
  }
}