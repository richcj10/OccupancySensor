#ifndef SENSOR_H
#define SENSOR_H

#define SENSOR_INPUT_TEMP1 1
#define SENSOR_INPUT_TEMP2 2
#define SENSOR_INPUT_HUMID1 3
#define SENSOR_INPUT_HUMID2 4
#define SENSOR_INPUT_PRES 5
#define SENSOR_INPUT_LUX 6
#define SENSOR_INPUT_VIN 7
#define SENSOR_INPUT_MOTION 8

void SensorStart();
void ReadSensors();
float GetSensorValues(char Type);
void UpdatePixels(unsigned char R, unsigned char G, unsigned char B);
void SampleVin();
void DebounceMotionSensor();

#endif