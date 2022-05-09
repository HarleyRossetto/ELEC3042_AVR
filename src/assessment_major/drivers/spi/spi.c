#include "spi.h"
#include "avr/io.h"

inline Initialiser initialiseSPIAsMaster() {
    DDRB |= (1 << DDB3) | (1 << DDB5);
    SPCR = (1 << SPE) | (1 << MSTR) | SPI_CLOCK_RATE_2;
    SPSR = (1 << SPI2X);
    
    // Enable Chip Select as output
    DDRB |= (1 << DDB2);
    PORTB |= (1 << PB2); // Raise CS
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