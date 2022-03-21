#include "avr/io.h"

uint8_t spiTransfer(uint8_t data) {
    
}


int main(void) {
    /**
     * SPI Control Register
     * 
     *|  7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
     * -------------------------------------------------------------- 
     *| SPIE | SPE   | DORD  | MSTR  | CPOL  | CPHA  | SPR1  | SPR0  |
     * 
     * SPIE:    Spi Interrupt Enable
     *          1 = Enabled
     * 
     * SPE:     SPI Enable
     *          1 = SPI Enabled
     * 
     * DORD:    Data Order
     *          1 = LST of data word is transmitted first.
     *          0 = MSB of data word is transmitted first.
     * 
     * MSTR:    Master/Slave Select
     *          1 = Master Mode
     *          0 = Slave Mode
     * 
     * CPOL:    Clock Polarity
     *          |    CPOL    |   Leading Edge    |   Trailing Edge   |
     *          |     0      |       Rising      |       Falling     |
     *          |     1      |       Falling     |       Rising      |
     * 
     * CPHA:    Clock Phase
     *          |    CPHA    |   Leading Edge    |   Trailing Edge   |
     *          |     0      |       Sample      |       Setup       |
     *          |     1      |       Setup       |       Sample      |
     * 
     * 
     * SPR0/1:  SPI Clock Rate Select 1 and 0
     * 
     */
    SPSR = 0x0;
    return 0;
}