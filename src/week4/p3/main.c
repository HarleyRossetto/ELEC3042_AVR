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

#define WIN_TONE 128
#define LOSE_TONE 220;

void initTimer() {
    TCCR1A = (1 << COM1A0);
    TCCR1B = (1 << CS11) | (1 << WGM12);
    OCR1A = WIN_TONE;
    OCR1B = LOSE_TONE;
    TIMSK1 = 0;
    TIFR1 = 0;
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
    initTimer();
    //DDRB |= (1 << PB1);

    DDRB = 0b00011111;
    PORTB = (1 << PB1);
    
    DDRC = 0x0;
    PORTC = 0x0;

    DDRD = 0b11111000;
    PORTD = (1 << PD2);
}

uint16_t leds = (1 << 3);

#define PORTB_BIT_MSK 0x7
#define PORTD_BIT_MSK 0x1F8
#define PORT_D_BITS(input) (input & PORTD_BIT_MSK)
#define PORT_B_BITS(input) (input & PORTB_BIT_MSK)

#define GREEN_LIT (leds & (1 << 7))

void showLeds(uint16_t input) { 
    PORTD = (1 << PD2) | (input);
    PORTB = 0;
    if (input == 0x100)
      PORTB = (input >> 8);
    else if (input > 0x100) {
      PORTB = (input >> 7);
    }
}

uint8_t direction = 1;

void updateLeds() {
    //Manage led bit shift
    if (direction)
        leds = leds << 1;
    else
      leds = leds >> 1;

    //Manage direction changes
    if (leds & (1 << 11))
      direction = 0;
    else if (leds & (1 << 3))
      direction = 1;
}

#define BUTTON_PRESSED !(PIND & (1 << PD2))

typedef enum {
    Play,
    Win,
    Try,
    Miss,
    Restart
} GameState;

int main(void) {
  setup(); // set up the physical hardware
  int delay = 20;

  GameState state = Play;

  while (1) {
    // PORTB ^= _BV(1);
    delay_ms(delay);

    showLeds(leds);

    switch (state)
    {
        case Play:
          if (BUTTON_PRESSED && GREEN_LIT) {
            // Win state
          } else if (BUTTON_PRESSED) {

          }else {
            updateLeds();
          }
            break;
        case Win:
            /* code */
            break;
        case Try:
            /* code */
            break;
        case Miss:
            /* code */
            break;
        case Restart:
            /* code */
            break;
    
    default:
        break;
    }
  }
}