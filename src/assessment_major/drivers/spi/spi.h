#ifndef SPI_H
#define SPI_H

#include "stdint.h"
#include "../../types.h"
#include "../../iotypes.h"

// Sparkfun can_bus Library
// https://github.com/sparkfun/SparkFun_CAN-Bus_Arduino_Library/blob/master/src/global.h
#define CLEAR_BIT(x) _CLEAR_BIT(x)
#define SET_BIT(x) _SET_BIT(x)
#define _CLEAR_BIT(x,y) PORT(x) &= ~(1 << y)
#define _SET_BIT(x,y) PORT(x) |= (1 << y)
#define	PORT(x)	PORT ## x
// Sparkfun can_bus Library

#define SPI_CLOCK_RATE_2    SPI_CLOCK_RATE_4
#define SPI_CLOCK_RATE_4    (0 << SPR1) | (0 << SPR0)
#define SPI_CLOCK_RATE_16   (0 << SPR1) | (1 << SPR0)
#define SPI_CLOCK_RATE_64   (1 << SPR1) | (0 << SPR0)
#define SPI_CLOCK_RATE_128  (1 << SPR1) | (1 << SPR0)

Initialiser initialiseSPIAsMaster();
void SPI_EnableTransferCompleteInterrupt();
void SPI_DisableTransferCompleteInterrupt();

uint8_t SPI_ShiftByte(uint8_t byte);

void SPI_ShiftByte_Async(uint8_t byte);

#endif //SPI_H