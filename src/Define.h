#ifndef DEFINE_H
#define DEFINE_H

//IO DEFINE
#define VOLTSENSE A3
#define DIR 4
#define MOTION 2
#define IR 3
#define LED 7

//SENSOR DEFINE
#define OCCUPYSENSORADDRESS 0x11
#define OCCUPYSENSORTYPE 12

#define SENSOR_CHANGE_ADDRESS 0x10
#define SENSOR_CHANGE_SCANRATE 0x11
#define SENSOR_CHANGE_FAULT 0x12
#define SENSOR_CHANGE_SENSOR_OPEN 0x13
#define SENSOR_SET_LED 20

#define SENSOR_SCAN_EN 0//Sensor Scan Enable
#define SENSOR_LED_CONTROL 1//LED Control
#define SENSOR_MOTION 2 //Auto Relay Control - Default ON
#define SENSOR_RELAY 3 //Relay Control - Default OFF

#define SENSORSCANRATE 2
#define SENSORTESTTIMEMULTIPLYER 1000

#define SENSORHIGHFAULT 3
#define SENSORFAULT 2
#define SENSORWATERDETECT 1
#define SENSOROK 0

#endif