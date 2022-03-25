/*
 * File:   22S1_ELEC3042_L5_sample.c
 * Author: rex
 *
 * Created on 5 February 2022, 10:42 AM
 *
 * Sample code for I2C LCD display.
 */

#include "22S1_ELEC3042_I2C_PCF8574.h"
#include "avr/interrupt.h"
#include "avr/io.h"
#include "stdint.h"

typedef enum { Play, Win, Try, Miss, Restart } GameState;

typedef uint8_t bool;
#define true 1
#define false 0

#define BUTTON_PRESSED !(PIND & (1 << PD2))

#define WIN_TONE 128
#define LOSE_TONE 220;

uint16_t leds = (1 << 3);
uint8_t direction = 1;

#define PORTB_BIT_MSK 0x7
#define PORTD_BIT_MSK 0x1F8
#define PORT_D_BITS(input) (input & PORTD_BIT_MSK)
#define PORT_B_BITS(input) (input & PORTB_BIT_MSK)

#define GREEN_LIT (leds & (1 << 7))

void showLeds(uint16_t input);

void updateLeds() {
  // Manage led bit shift
  if (direction)
    leds = leds << 1;
  else
    leds = leds >> 1;

  // Manage direction changes
  if (leds & (1 << 11))
    direction = 0;
  else if (leds & (1 << 3))
    direction = 1;
}

#define CLOCK_SELECT_NO_SOURCE (0 << CS12) | (0 << CS11) | (0 << CS10)
#define CLOCK_SELECT_1_PRESCALER (0 << CS12) | (0 << CS11) | (1 << CS10)
#define CLOCK_SELECT_8_PRESCALER (0 << CS12) | (1 << CS11) | (0 << CS10)
#define CLOCK_SELECT_64_PRESCALER (0 << CS12) | (1 << CS11) | (1 << CS10)
#define CLOCK_SELECT_256_PRESCALER (1 << CS12) | (0 << CS11) | (0 << CS10)
#define CLOCK_SELECT_1024_PRESCALER (1 << CS12) | (0 << CS11) | (1 << CS10)

void initTimer() {
  TCCR1A = (1 << COM1A0);
  TCCR1B = (1 << WGM12);
  OCR1A = WIN_TONE;
  OCR1B = LOSE_TONE;
  TIMSK1 = 0;
  TIFR1 = 0;
}

#define FLAGA 1
#define FLAGB 2
#define FLAGC 3
uint8_t flags = 0xff;

#define raised(flag) flags &(1 << flag)
#define clearFlag(flag) z;

void initTimer0() {
  TCCR0A = (1 << WGM01) | (0 << WGM00);
  TCCR0B = (0 << WGM02) | CLOCK_SELECT_64_PRESCALER;
  TCNT0 = 0;
  OCR0A = 250;
  OCR0B = 0;
  TIMSK0 = (1 << OCIE0A);
  TIFR0 = 0x00;
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
  initTimer0();
  // DDRB |= (1 << PB1);

  DDRB = 0b00011111;
  PORTB = (1 << PB1);

  DDRC = 0x0;
  PORTC = 0x0;

  DDRD = 0b11111000;
  PORTD = (1 << PD2);

  setup_I2C();
}

/**
 * Convert a 4 bit value to its hexadecimal equivalent. This is algorithmically
 * slow, but easy to understand code.
 *
 * @param value number 0-15 to display as hexadecimal
 * @return ASCII character for hexadecimal value
 */
char hex(int value) { return "0123456789ABCDEF"[value & 0x0f]; }

char lvl[] = "LEVEL:";
char press[] = "PRESS:";
char btn[] = "BTN:";
char hit[] = "HIT  ";
char close[] = "CLOSE";
char miss[] = "MISS ";

GameState state = Play;
bool updateDisplay = true;

volatile uint64_t tick = 0;
#define MOD 100
uint16_t LED_UPDATE_DELAY = 250;
#define WIN_DELAY 2500

int main(void) {
  uint8_t LCD_Addr = 0x27;

  setup(); // set up the physical hardware
  setup_LCD(LCD_Addr);

  sei();

  while (1) {
    showLeds(leds);

    switch (state) {
    case Play:
      if (BUTTON_PRESSED) {
        updateDisplay = true;
        if (leds == 128) {
          state = Win;
          tick = 0;
        } else if (leds == 64 || leds == 256) {
          state = Try;
          tick = 0;
        } else {
          state = Miss;
          tick = 0;
        }
      } else {
        if (tick == LED_UPDATE_DELAY) {
          updateLeds();
          tick = 0;
        }
      }
      break;
    case Win:
      /* code */
      TCCR1B |= CLOCK_SELECT_8_PRESCALER;
      if (tick == WIN_DELAY) {
        tick = 0;
        state = Restart;
      }
      break;
    case Try:
      /* code */
      updateDisplay = true;
      state = Restart;
      break;
    case Miss:
      /* code */
      updateDisplay = true;
      state = Restart;
      break;
    case Restart:
      /* code */
      updateDisplay = true;
      state = Play;
      leds = (1 << 3);
      TCCR1B &= 0xF0 | (CLOCK_SELECT_NO_SOURCE);
      break;

    default:
      break;
    }

    if (updateDisplay) {
      updateDisplay = false;
      LCD_Position(LCD_Addr, 0x00);

      LCD_Write(LCD_Addr, lvl, 6);
      LCD_Write(LCD_Addr, "00", 2);

      LCD_Position(LCD_Addr, 11);
      LCD_Write(LCD_Addr, btn, 4);
      LCD_Write(LCD_Addr, BUTTON_PRESSED ? "C" : "O", 1);

      LCD_Position(LCD_Addr, 0x40);
      LCD_Write(LCD_Addr, press, 6);

      if (BUTTON_PRESSED) {
        if (leds == 128) {
          LCD_Write(LCD_Addr, hit, 5);
        } else if (leds == 64 || leds == 256) {
          LCD_Write(LCD_Addr, close, 5);
        } else {
          LCD_Write(LCD_Addr, miss, 5);
        }
      } else {
        LCD_Write(LCD_Addr, "     ", 5);
      }
    }
  }
}

void showLeds(uint16_t input) {
  PORTD = (1 << PD2) | (input);
  PORTB = 0;
  if (input == 0x100)
    PORTB = (input >> 8);
  else if (input > 0x100) {
    PORTB = (input >> 7);
  }
}

ISR(TIMER0_COMPA_vect) { tick++; }
