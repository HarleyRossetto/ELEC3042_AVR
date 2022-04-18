#include "stdint.h"
#include "avr/io.h"
#include "avr/interrupt.h"
#include "bool.h"
#include "null.h"
#include "led.h"
#include "drivers/i2c/display/22S1_ELEC3042_I2C_PCF8574.h"
#include "drivers/spi/spi.h"
#include "drivers/mcp23s17/mcp23s17.h"
#include "flag.h"
#include "types.h"

/**
 * Initialiser Prototypes
 */
Initialiser initialise();
Initialiser initialiseFlags();

Action actionUpdateDisplay();

const uint8_t LCD_Addr = 0x27;

/**
 * Flag declarations
 */
Flag flag_UpdateDisplay;

LED debugLed;

Initialiser initialise() {
    setup_I2C();
    setup_LCD(LCD_Addr);

    // Enable Chip Select as output
    DDRB |= (1 << DDB2);
    PORTB |= (1 << PB2); // Raise CS

    initialiseSPIAsMaster(&DDRB, DDB3, DDB5);
    MCP23S17_Initialise(); // Must be initialised AFTER SPI.

    initialiseFlags();

    Flag_Set(&flag_UpdateDisplay);
    
    MCP23S17_WriteRegister(0x00, 0x00);
    MCP23S17_WriteRegister(GPIOA, 0b10001000);
}

Initialiser initialiseFlags() { flag_UpdateDisplay = Flag_Create(&actionUpdateDisplay, NULL); }

int main(void) {
    initialise();

    while (1) {
        // Flag_RunIfSet(&flag_UpdateDisplay);
        actionUpdateDisplay();
    }

    return 0;
}

Action actionUpdateDisplay() {
    uint8_t iodira = MCP23S17_ReadRegister(IOCON);
    

    LCD_Position(LCD_Addr, 0);
    LCD_Write(LCD_Addr, "IOCON:", 6);


    // char higher = ((iodira >> 4) & 0xf) + 48;
    // LCD_Write(LCD_Addr, &higher, 1);
    // char lower = (iodira & 0xf) + 48;
    // LCD_Write(LCD_Addr, &lower, 1);


    for (int i = 7; i >= 0; i-- ) {
        char c = ((iodira >> i) & 1) + 48;
        LCD_Write(LCD_Addr, &c, 1);
    }    
}