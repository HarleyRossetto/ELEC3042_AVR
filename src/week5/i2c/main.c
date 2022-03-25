/*
 * File:   22S1_ELEC3042_L5_sample.c
 * Author: rex
 *
 * Created on 5 February 2022, 10:42 AM
 * 
 * Sample code for I2C LCD display.
 */

#include "avr/io.h"
#include "stdint.h"
#include "22S1_ELEC3042_I2C_PCF8574.h"

/*
 * We set the Data direction register to specify which pins are inputs and
 * which are outputs. A 0 bit indicates the corresponding pin is an input,
 * a 1 indicates the corresponding pin is an output
 */
void setup() {
    DDRC = 0b00000000;
    PORTC = 0b00000000;
}

/**
 * Convert a 4 bit value to its hexadecimal equivalent. This is algorithmically
 * slow, but easy to understand code.
 * 
 * @param value number 0-15 to display as hexadecimal
 * @return ASCII character for hexadecimal value
 */
char hex(int value) {
    return "0123456789ABCDEF"[value & 0x0f];
}

int main(void) {
    char str1[] = "ELEC3042 2022.  ";
    char str2[] = "Address is 0x__ ";
    uint8_t LCD_Addr = 0x27;
    
    setup();    // set up the physical hardware
    setup_I2C();
    
    if (setup_LCD(LCD_Addr) == -1) {
        LCD_Addr = 0x3f;
        if (setup_LCD(LCD_Addr) == -1) {
            LCD_Addr = 0x20;
            if (setup_LCD(LCD_Addr) == -1) {
                while(1) {
                    PORTB ^= 0x20;  // flash the led as we don't know.
                }
            }
        }
    }
    LCD_Position(LCD_Addr, 0x00);
    LCD_Write(LCD_Addr, str1, 16);
    LCD_Position(LCD_Addr, 0x40);
    LCD_Write(LCD_Addr, str2, 16);
    
    LCD_Position(LCD_Addr, 0x4D);
    LCD_Write_Chr(LCD_Addr, hex(LCD_Addr>>4));
    LCD_Write_Chr(LCD_Addr, hex(LCD_Addr));
    while (1) {
        ; //do nothing
    }
}
