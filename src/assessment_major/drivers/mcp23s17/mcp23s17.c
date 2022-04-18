#include "mcp23s17.h"
#include "../spi/spi.h"
#include "avr/io.h"

#define CS        B, PB2

#define MCP23S17  64
#define DEVICE_ID 0

Initialiser MCP23S17_Initialise() {
    // Initialise the port expander so that interrupt pins are mirrored and sequential operations are initially disabled.
    // If we want to use BANK = 1, set accordingly.
#if defined (BANK_MODE_ONE)
    MCP23S17_WriteRegister(IOCON, (1 << BANK) | (1 << MIRROR) | (1 << SEQOP));
#else
    MCP23S17_WriteRegister(IOCON, (1 << MIRROR) | (1 << SEQOP));
#endif
}

void MCP23S17_WriteRegister(uint8_t address, uint8_t data) {
    CLEAR_BIT(CS);

    SPI_ShiftByte(MCP23S17 | DEVICE_ID | WRITE);
    SPI_ShiftByte(address);
    SPI_ShiftByte(data);

    SET_BIT(CS);
}

uint8_t MCP23S17_ReadRegister(uint8_t address) {
    CLEAR_BIT(CS);

    SPI_ShiftByte(MCP23S17 | DEVICE_ID | READ);
    SPI_ShiftByte(address);
    uint8_t result = SPI_ShiftByte(0x00);

    SET_BIT(CS);

    return result;
}
