#include "spi.h"
#include "avr/io.h"

inline Initialiser initialiseSPIAsMaster(DDR spiDdr, Pin mosi, Pin sck) {
    *spiDdr |= (1 << mosi) | (1 << sck);
    SPCR = (1 << SPE) | (1 << MSTR) | SPI_CLOCK_RATE_16;
    SPSR = 0;
}

inline void SPI_EnableTransferCompleteInterrupt() { SPCR |= (1 << SPIE); }

inline void SPI_DisableTransferCompleteInterrupt() { SPCR &= ~(1 << SPIE); }

uint8_t SPI_ShiftByte(uint8_t data) {
    SPDR = data;
    
    while (!(SPSR & (1 << SPIF)))
        ;

    return SPDR;
}

void SPI_ShiftByte_Async(uint8_t data) { SPDR = data; }