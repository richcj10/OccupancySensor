#ifndef SIMPLE_MODBUS_SLAVE_H
#define SIMPLE_MODBUS_SLAVE_H

/*
 * SimpleModbusSlave — forked and extended for ModBusBL.
 *
 * Changes from original (Juan Bester):
 *   - Added FC65 (0x41) bootloader-trigger handler.
 *   - Added modbus_register_bootloader_trigger() for callback registration.
 *   - Minimum frame check relaxed to >=6 bytes to support 6-byte FC65 frames.
 *
 * Original: Juan Bester <bester.juan@gmail.com>
 * ModBusBL fork: see ModBusBL.h
 */

#include "Arduino.h"
#include "MBBP.h"

void modbus_configure(long baud, byte _slaveID, byte _TxEnablePin,
                      unsigned int _holdingRegsSize, unsigned int _InputregsSize,
                      unsigned int _CoilregsSize, unsigned int _DiscreteInputregsSize,
                      unsigned char _lowLatency);

unsigned int modbus_update(unsigned int *holdingRegs, unsigned int *inputregs,
                           bool *coilregs, bool *DiscreteInputregs);

/* Register a callback invoked when a valid FC65 frame is received.
   The callback fires AFTER the ACK response is sent. */
void modbus_register_bootloader_trigger(void (*callback)(void));

#endif
