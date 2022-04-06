#ifndef EEPROM_H
#define EEPROM_H

#include "stdint.h"
#include "avr/io.h"

/*
    isReady
*/

uint8_t EEPROM_ReadByte(uint8_t *addr);
uint8_t EEPROM_UpdateByte(uint8_t *addr, uint8_t value);



#endif //EEPROM_H