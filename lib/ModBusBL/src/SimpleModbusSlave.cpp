#include "SimpleModbusSlave.h"
#include <math.h>

#define BUFFER_SIZE 128
#define BitSet(arg,posn) ((arg) | (1L << (posn)))

static unsigned char frame[BUFFER_SIZE];
static unsigned int  holdingRegsSize;
static unsigned int  InputRegsSize;
static unsigned int  CoilRegsSize;
static unsigned int  DiscreateInputSize;
static unsigned char broadcastFlag;
static unsigned char slaveID;
static unsigned char function;
static unsigned char TxEnablePin;
static unsigned int  errorCount;
static unsigned int  T1_5;
static unsigned int  T3_5;
static unsigned char noOfBytes = 0;
static boolean       BoolReturnData[20];
static unsigned char ReturnData[10];
static unsigned char ForLoopCounter = 0;

static void (*_bootloader_callback)(void) = NULL;

static void exceptionResponse(unsigned char exception);
static unsigned int calculateCRC(unsigned char bufferSize);
static void sendPacket(unsigned char bufferSize);

void modbus_register_bootloader_trigger(void (*callback)(void)) {
    _bootloader_callback = callback;
}

unsigned int modbus_update(unsigned int *holdingRegs, unsigned int *inputregs,
                           bool *coilregs, bool *DiscreteInputregs)
{
    unsigned char buffer   = 0;
    unsigned char overflow = 0;

    while (Serial.available()) {
        if (overflow)
            Serial.read();
        else {
            if (buffer == BUFFER_SIZE) overflow = 1;
            frame[buffer] = Serial.read();
            buffer++;
        }
        delayMicroseconds(T1_5);
    }

    if (overflow)
        return errorCount++;

    /* FC65 trigger frame is 6 bytes — handle before the ">6" general check */
    if (buffer == 6) {
        unsigned char id = frame[0];
        if (id == slaveID) {
            unsigned int crc = ((frame[4] << 8) | frame[5]);
            if (calculateCRC(4) == crc && frame[1] == MBBP_FC_TRIGGER) {
                if (frame[2] == MBBP_MAGIC_H && frame[3] == MBBP_MAGIC_L) {
                    /* ACK: echo [ADDR][FC65][0x00][0x00][CRC] */
                    frame[0] = slaveID;
                    frame[1] = MBBP_FC_TRIGGER;
                    frame[2] = 0x00;
                    frame[3] = 0x00;
                    unsigned int crc16 = calculateCRC(4);
                    frame[4] = crc16 >> 8;
                    frame[5] = crc16 & 0xFF;
                    sendPacket(6);
                    if (_bootloader_callback) _bootloader_callback();
                } else {
                    exceptionResponse(3); /* ILLEGAL DATA VALUE — wrong magic */
                }
            } else {
                errorCount++;
            }
        }
        return errorCount;
    }

    if (buffer > 6) {
        unsigned char id = frame[0];
        broadcastFlag = 0;
        if (id == 0) broadcastFlag = 1;

        if (id == slaveID || broadcastFlag) {
            unsigned int crc = ((frame[buffer - 2] << 8) | frame[buffer - 1]);
            if (calculateCRC(buffer - 2) == crc) {
                function = frame[1];
                unsigned int startingAddress  = ((frame[2] << 8) | frame[3]);
                unsigned int no_of_registers  = ((frame[4] << 8) | frame[5]);
                unsigned int maxData          = startingAddress + no_of_registers;
                unsigned char index;
                unsigned char address;
                unsigned int  crc16;

                if (!broadcastFlag && (function == 1)) {
                    if (startingAddress < CoilRegsSize) {
                        if (maxData <= CoilRegsSize) {
                            if (no_of_registers < 8)
                                noOfBytes = 1;
                            else {
                                noOfBytes = no_of_registers / 8;
                                if ((no_of_registers % 8) != 0) noOfBytes++;
                            }
                            ForLoopCounter = startingAddress;
                            unsigned char BitCounter  = 0;
                            unsigned char ByteCounter = 0;
                            for (; ForLoopCounter < no_of_registers; ForLoopCounter++) {
                                if (coilregs[ForLoopCounter] == 1)
                                    ReturnData[ByteCounter] = BitSet(ReturnData[ByteCounter], BitCounter);
                                BitCounter++;
                                if (BitCounter > 7) { BitCounter = 0; ByteCounter++; }
                            }
                            unsigned char responseFrameSize = 5 + noOfBytes;
                            frame[0] = slaveID;
                            frame[1] = function;
                            frame[2] = noOfBytes;
                            address  = 3;
                            for (unsigned char jk = 0; jk <= noOfBytes; jk++) {
                                frame[address] = ReturnData[jk];
                                address++;
                                ReturnData[jk] = 0;
                            }
                            crc16 = calculateCRC(responseFrameSize - 2);
                            frame[responseFrameSize - 2] = crc16 >> 8;
                            frame[responseFrameSize - 1] = crc16 & 0xFF;
                            sendPacket(responseFrameSize);
                        } else exceptionResponse(3);
                    } else exceptionResponse(2);
                }
                else if (!broadcastFlag && (function == 2)) {
                    if (startingAddress < DiscreateInputSize) {
                        if (maxData <= DiscreateInputSize) {
                            if (no_of_registers < 8)
                                noOfBytes = 1;
                            else {
                                noOfBytes = no_of_registers / 8;
                                if ((no_of_registers % 8) != 0) noOfBytes++;
                            }
                            ForLoopCounter = startingAddress;
                            unsigned char BitCounter  = 0;
                            unsigned char ByteCounter = 0;
                            for (; ForLoopCounter < no_of_registers; ForLoopCounter++) {
                                if (DiscreteInputregs[ForLoopCounter] == 1)
                                    ReturnData[ByteCounter] = BitSet(ReturnData[ByteCounter], BitCounter);
                                BitCounter++;
                                if (BitCounter > 7) { BitCounter = 0; ByteCounter++; }
                            }
                            unsigned char responseFrameSize = 5 + noOfBytes;
                            frame[0] = slaveID;
                            frame[1] = function;
                            frame[2] = noOfBytes;
                            address  = 3;
                            for (unsigned char jk = 0; jk <= noOfBytes; jk++) {
                                frame[address] = ReturnData[jk];
                                address++;
                                ReturnData[jk] = 0;
                            }
                            crc16 = calculateCRC(responseFrameSize - 2);
                            frame[responseFrameSize - 2] = crc16 >> 8;
                            frame[responseFrameSize - 1] = crc16 & 0xFF;
                            sendPacket(responseFrameSize);
                        } else exceptionResponse(3);
                    } else exceptionResponse(2);
                }
                else if (!broadcastFlag && (function == 3)) {
                    if (startingAddress < holdingRegsSize) {
                        if (maxData <= holdingRegsSize) {
                            noOfBytes = no_of_registers * 2;
                            unsigned char responseFrameSize = 5 + noOfBytes;
                            frame[0] = slaveID;
                            frame[1] = function;
                            frame[2] = noOfBytes;
                            address  = 3;
                            unsigned int temp;
                            for (index = startingAddress; index < maxData; index++) {
                                temp = holdingRegs[index];
                                frame[address] = temp >> 8; address++;
                                frame[address] = temp & 0xFF; address++;
                            }
                            crc16 = calculateCRC(responseFrameSize - 2);
                            frame[responseFrameSize - 2] = crc16 >> 8;
                            frame[responseFrameSize - 1] = crc16 & 0xFF;
                            sendPacket(responseFrameSize);
                        } else exceptionResponse(3);
                    } else exceptionResponse(2);
                }
                else if (!broadcastFlag && (function == 4)) {
                    if (startingAddress < InputRegsSize) {
                        if (maxData <= InputRegsSize) {
                            noOfBytes = no_of_registers * 2;
                            unsigned char responseFrameSize = 5 + noOfBytes;
                            frame[0] = slaveID;
                            frame[1] = function;
                            frame[2] = noOfBytes;
                            address  = 3;
                            unsigned int temp;
                            for (index = startingAddress; index < maxData; index++) {
                                temp = inputregs[index];
                                frame[address] = temp >> 8; address++;
                                frame[address] = temp & 0xFF; address++;
                            }
                            crc16 = calculateCRC(responseFrameSize - 2);
                            frame[responseFrameSize - 2] = crc16 >> 8;
                            frame[responseFrameSize - 1] = crc16 & 0xFF;
                            sendPacket(responseFrameSize);
                        } else exceptionResponse(3);
                    } else exceptionResponse(2);
                }
                else if (!broadcastFlag && (function == 5)) {
                    if (startingAddress < CoilRegsSize) {
                        if (frame[4] == 0xFF)
                            coilregs[startingAddress] = 1;
                        else
                            coilregs[startingAddress] = 0;
                        sendPacket(8);
                    } else exceptionResponse(2);
                }
                else if (function == 6) {
                    if (startingAddress < holdingRegsSize) {
                        unsigned int regStatus = ((frame[4] << 8) | frame[5]);
                        holdingRegs[startingAddress] = regStatus;
                        crc16 = calculateCRC(6);
                        frame[6] = crc16 >> 8;
                        frame[7] = crc16 & 0xFF;
                        sendPacket(8);
                    } else exceptionResponse(2);
                }
                else if (!broadcastFlag && (function == 15)) {
                    if (startingAddress < CoilRegsSize) {
                        unsigned char responseFrameSize = 8;
                        crc16 = calculateCRC(responseFrameSize - 2);
                        frame[responseFrameSize - 2] = crc16 >> 8;
                        frame[responseFrameSize - 1] = crc16 & 0xFF;
                        sendPacket(responseFrameSize);
                    } else exceptionResponse(2);
                }
                else if (function == 16) {
                    if (frame[6] == (buffer - 9)) {
                        if (startingAddress < holdingRegsSize) {
                            if (maxData <= holdingRegsSize) {
                                address = 7;
                                for (index = startingAddress; index < maxData; index++) {
                                    holdingRegs[index] = ((frame[address] << 8) | frame[address + 1]);
                                    address += 2;
                                }
                                crc16 = calculateCRC(6);
                                frame[6] = crc16 >> 8;
                                frame[7] = crc16 & 0xFF;
                                if (!broadcastFlag) sendPacket(8);
                            } else exceptionResponse(3);
                        } else exceptionResponse(2);
                    } else errorCount++;
                }
                else {
                    exceptionResponse(1); /* ILLEGAL FUNCTION */
                }
            } else {
                errorCount++; /* CRC mismatch */
            }
        }
    } else if (buffer > 0 && buffer < 6) {
        errorCount++; /* too short */
    }

    return errorCount;
}

static void exceptionResponse(unsigned char exception) {
    errorCount++;
    if (!broadcastFlag) {
        frame[0] = slaveID;
        frame[1] = (function | 0x80);
        frame[2] = exception;
        unsigned int crc16 = calculateCRC(3);
        frame[3] = crc16 >> 8;
        frame[4] = crc16 & 0xFF;
        sendPacket(5);
    }
}

void modbus_configure(long baud, byte _slaveID, byte _TxEnablePin,
                      unsigned int _holdingRegsSize, unsigned int _InputregsSize,
                      unsigned int _CoilregsSize, unsigned int _DiscreteInputregsSize,
                      unsigned char _lowLatency)
{
    slaveID = _slaveID;
    Serial.begin(baud);

    if (_TxEnablePin > 1) {
        TxEnablePin = _TxEnablePin;
        pinMode(TxEnablePin, OUTPUT);
        digitalWrite(TxEnablePin, LOW);
    }

    if (baud == 1000000 && _lowLatency) {
        T1_5 = 1;   T3_5 = 10;
    } else if (baud >= 115200 && _lowLatency) {
        T1_5 = 75;  T3_5 = 175;
    } else if (baud > 19200) {
        T1_5 = 750; T3_5 = 1750;
    } else {
        T1_5 = 15000000 / baud;
        T3_5 = 35000000 / baud;
    }

    holdingRegsSize  = _holdingRegsSize;
    InputRegsSize    = _InputregsSize;
    CoilRegsSize     = _CoilregsSize;
    DiscreateInputSize = _DiscreteInputregsSize;
    errorCount = 0;
}

static unsigned int calculateCRC(byte bufferSize) {
    unsigned int temp, temp2, flag;
    temp = 0xFFFF;
    for (unsigned char i = 0; i < bufferSize; i++) {
        temp = temp ^ frame[i];
        for (unsigned char j = 1; j <= 8; j++) {
            flag = temp & 0x0001;
            temp >>= 1;
            if (flag) temp ^= 0xA001;
        }
    }
    temp2 = temp >> 8;
    temp  = (temp << 8) | temp2;
    temp &= 0xFFFF;
    return temp;
}

static void sendPacket(unsigned char bufferSize) {
    if (TxEnablePin > 1) digitalWrite(TxEnablePin, HIGH);
    delayMicroseconds(500);
    for (unsigned char i = 0; i < bufferSize; i++)
        Serial.write(frame[i]);
    Serial.flush();
    delayMicroseconds(T3_5);
    if (TxEnablePin > 1) digitalWrite(TxEnablePin, LOW);
}
