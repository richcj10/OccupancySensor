#include <Arduino.h>
#include "ModBusBL.h"

#include "EEPROMData.h"
#include "Sensor.h"
#include "Define.h"

unsigned long previousMillis = 0;
unsigned long previousLEDMillis = 0;
unsigned int SensorTestTime = 3;

// DIR=4, 3 user holding regs, 9 input regs, 4 coils, 0 DI
ModBusBL modbus(DIR, 3, 9, 4, 0);

void ComunicationUpdate() {
    modbus.update();

    long holdingRegisterValue = modbus.getHolding(0); // bus holding reg 1
    if (holdingRegisterValue == SENSOR_CHANGE_ADDRESS) {
        SetAddressFromEEPROM((char)modbus.getHolding(0));
    } else if (holdingRegisterValue == SENSOR_CHANGE_SCANRATE) {
        SetScanRateFromEEPROM((char)modbus.getHolding(0));
    } else if (holdingRegisterValue == SENSOR_SET_LED) {
        int High = modbus.getHolding(1); // bus holding reg 2
        int Low  = modbus.getHolding(2); // bus holding reg 3
        UpdatePixels(lowByte(High), highByte(Low), lowByte(Low));
    }

    modbus.setInput(2, GetSensorValues(SENSOR_INPUT_VIN)    * 100);
    modbus.setInput(3, GetSensorValues(SENSOR_INPUT_TEMP1)  * 100);
    modbus.setInput(4, GetSensorValues(SENSOR_INPUT_TEMP2)  * 100);
    modbus.setInput(5, GetSensorValues(SENSOR_INPUT_HUMID1) * 100);
    modbus.setInput(6, GetSensorValues(SENSOR_INPUT_HUMID2) * 100);
    modbus.setInput(7, GetSensorValues(SENSOR_INPUT_PRES));
    modbus.setInput(8, GetSensorValues(SENSOR_INPUT_LUX));

    modbus.setCoil(SENSOR_MOTION, GetSensorValues(SENSOR_INPUT_MOTION));
}

void setup() {
    pinMode(LED, OUTPUT);
    pinMode(MOTION, INPUT);
    pinMode(IR, INPUT);

    // begin() reads slave ID from EEPROM[0x01], defaults to MBBP_SLAVE_ADDR (0x10).
    // Also initialises Serial and the RS-485 DIR pin internally.
    modbus.begin(38400);

    // reg 0: combined identity word — master validates via swVersion = (type<<8)|version
    modbus.setInput(0, FIRMWARE_VERSION);
    modbus.setInput(1, OCCUPYSENSORTYPE);
    modbus.setCoil(SENSOR_SCAN_EN,    true);
    modbus.setCoil(SENSOR_LED_CONTROL, true);

    char ScanRate = GetScanRateFromEEPROM();
    SensorTestTime = (ScanRate != -1) ? ScanRate : SENSORSCANRATE;

    SensorStart();
    UpdatePixels(50, 0, 0);  delay(100);
    UpdatePixels(0, 50, 0);  delay(100);
    UpdatePixels(0, 0, 50);  delay(100);
    UpdatePixels(0, 0, 0);
}

void loop() {
    ComunicationUpdate();
    unsigned long currentMillis = millis();
    if (modbus.getCoil(SENSOR_SCAN_EN)) {
        if (currentMillis - previousMillis >= SensorTestTime * SENSORTESTTIMEMULTIPLYER) {
            previousMillis = currentMillis;
            ReadSensors();
        }
    }
    if (modbus.getCoil(SENSOR_LED_CONTROL)) {
        if (currentMillis - previousLEDMillis >= 500) {
            previousLEDMillis = currentMillis;
            digitalWrite(LED, !digitalRead(LED));
        }
    }
}
