#ifndef EEPROM_ALARM_H
#define EEPROM_ALARM_H

#include "stdint.h"
#include "avr/io.h"
#include "avr/interrupt.h"
#include "bool.h"

void EEPROM_Read(uint16_t address, uint8_t *dataOut);
void EEPROM_Update(uint8_t data, uint16_t address);

#endif // EEPROM_ALARM_H