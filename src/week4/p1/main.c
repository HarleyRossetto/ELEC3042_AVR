/*
 * File:   22S1_ELEC3042_L4P1_speaker.c
 * Author: rex
 *
 * Created on 4 February 2022, 4:50 PM
 */

#include "avr/io.h"

/**
 * The delay routine is calibrated for an Arduino Uno 16MHz
 * 
 * @param num number of ms to delay
 */
void delay_ms(long num) {
    while (num--) {
        for (volatile long x = 0; x < 468; x++) {
            ;
        }
    }
}

/*
 * We set the Data direction register to specify which pins are inputs and
 * which are outputs. A 0 bit indicates the corresponding pin is an input,
 * a 1 indicates the corresponding pin is an output
 * 
 * For the speaker we use PB1.
 * PD2 is the push button input,
 * and PD3-7, PB0,2-4 are the LEDs
 */
void setup() {
    DDRB = 0b00011111;
    PORTB = 0b00011111;
    DDRC = 0b00000000;
    PORTC = 0b00000000;
    DDRD = 0b11111000;
    PORTD = 0b11111100;
}

int main(void) {
    setup();    // set up the physical hardware
    int delay = 4;
    while (1) {
        PORTB ^= _BV(1);
        delay_ms(delay);
        if ((PIND & _BV(2)) == 0) {
            PORTD |= 0b11111000;
            PORTB |= 0b00011101;
        } else {
            PORTD &= ~0b11111000;
            PORTB &= ~0b00011101;
        }
    }
}