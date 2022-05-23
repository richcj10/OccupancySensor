#ifndef SENSOR_H
#define SENSOR_H

#define SENSOR_BIAS 0
#define SENSOR_INPUT_1 1
#define SENSOR_INPUT_2 2
#define SENSOR_INPUT_3 3

void SensorStart();
void ReadSensors();
int GetSensorValues(char Type);
void UpdatePixels(unsigned char R, unsigned char G, unsigned char B);

#endif