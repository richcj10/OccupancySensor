#include "Sensor.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>

#include "Define.h"

#define PIN        6 // On Trinket or Gemma, suggest changing this to 1

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS 2 // Popular NeoPixel ring size

// When setting up the NeoPixel library, we tell it how many pixels,
// and which pin to use to send signals. Note that for older NeoPixel
// strips you might need to change the third parameter -- see the
// strandtest example for more information on possible values.
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

void SensorStart(){
  pixels.begin();
  pixels.clear();

  Wire.begin();
}

void ReadSensors(){

}

void UpdatePixels(unsigned char R, unsigned char G, unsigned char B){
    pixels.setPixelColor(0, pixels.Color(R, G, B));
    pixels.setPixelColor(1, pixels.Color(R, G, B));
    pixels.show();
}

int GetSensorValues(char Type){
  return 0;
}
