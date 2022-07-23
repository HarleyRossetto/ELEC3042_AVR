/*
 * File:   22S1_ELEC3042_L5P1_boppit.c
 * Author: rex
 *
 * Created on 5 February 2022, 11:23 AM
 * 
 * Update the Boppit game with the LCD code
 */

#include <xc.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "22S1_ELEC3042_I2C_PCF8574.h"
#include "22S1_ELEC3042_SPI.h"

volatile uint32_t   clock_count = 0;
// define a type with two values, pressed and released to hold the button state
enum BUTTON { Pressed, Released };
volatile enum BUTTON button_state = Released; // volatile as it is used in an interrupt
enum BUTTON oldButton_state = Released;
enum PRESS { None, Hit, Miss, Close };
enum PRESS press = None;
enum PRESS oldPress = None;

ISR(INT0_vect) {
    button_state = (PIND & 0b00000100)?Released:Pressed;
    if (button_state == Pressed) {
        PORTB |= 0x20;  // turn on inbuilt LED
    } else {
        PORTB &= 0xdf;  // turn off inbuilt LED
    }
}

uint32_t millis() {
    /*
     * Return the current clock_count value.
     * We temporarily disable interrupts to ensure the clock_count value
     * doesn't change while we are reading them.
     * 
     * We then restore the original SREG, which contains the I flag.
     */
    register uint32_t count;
    register char cSREG;
    
    cSREG = SREG;
    cli();
    count = clock_count;
    SREG = cSREG;
    return count;
}

/*
 * We interrupt 1000 times a second,
 * so clock_count increments once every millisecond.
 */
ISR(TIMER2_COMPA_vect) {
    clock_count++;
}

void setupTimer1() {
    /*
     * we want to send a square wave out OC1A (Port B Pin 1)
     * We have a 16MHz crystal, so the main clock is 16MHz.
     * We want to generate tones about the 1kHz range.
     * 
     * We will initially divide the clock by 8 (CS12/CS11/CS10 = 010)
     * which gives us a 2MHz clock. The timer divides this signal by
     * up to 65536 which gives us the final range of frequencies: 15Hz to 1MHz
     * which covers the range we desire.
     */
    DDRB |= _BV(1);         // set PORT B Pin 1 as an output
    PORTB &= ~_BV(1);       // set output to low
    /*
     * We will toggle Port B Pin 1 on timer expiry, clock /8, WGM=4 (CTC mode)
     */
    TCCR1A = 0b01000000;    // COM1A0 = 1
    TCCR1B = 0b00001000;    // noise = 0, clock is currently stopped.
    TCCR1C = 0b00000000;    // no force output
    /*
     * We set the frequency on OCR1A. When the counter matches this value the
     * output pin will toggle (1->0, 0->1). In this example we want the note
     * frequency to be 440Hz ('A'). This means the output has to toggle at
     * 880Hz. The clock is 2000000Hz, so 2000000/880 = 2273.
     * 
     * We can do the same calculation for whatever frequency we want.
     */
    OCR1A = 2273;           // 440Hz = Middle A
}

void setupTimer2() {
    /*
     * Timer 2 is setup as a millisecond interrupting timer.
     * This generates a regular millisecond interrupt,
     * which we can use internally to create a counter.
     * 
     * This counter is the basis of all internal timing.
     * 
     * The main clock is 16MHz, we need a 1kHz interrupt, so
     * the counter needs to count 16000 clock pulses and then interrupt.
     * 
     * The counter can count to 256, with prescalers of 8, 32, 64, 128, 256 or 1024
     * 
     * Using a prescaler of 128 we need a count of 125.
     * We want the prescaler to be as large as possible,
     * as it uses less power then the main counter.
     */
    TCCR2B = 0;             // turn off counter to configure
    TCNT2 = 0;              // set current count to zero
    OCR2A = 125;            // 125 * 128 * 1000 = 16000000
    TIFR2 = 0b00000111;     // clear all existing interrupts
    TIMSK2 = 0b00000010;    // enable interrupt on OCRA
    ASSR = 0;               // no async counting
    TCCR2A = 0b00000010;    // No I/O, mode = 2 (CTC)
    TCCR2B = 0b00000101;    // clock/128, mode = 2 (CTC), start counter
}

void startTone1() {
    OCR1A = 2273;
    TCCR1B = 0b00001010;
}

void startTone2() {
    OCR1A = 22222;
    TCCR1B = 0b00001010;
}

void stopTones() {
    OCR1A = 0;
    TCCR1B = 0b00001000;
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

int buttonPressed() {
    return ((SPI_Read_Command(0x13) & _BV(7)) == 0);
}

void write_LEDs(uint16_t leds) {
    SPI_Send_Command(0x14, (leds & 0xff));
    SPI_Send_Command(0x15, (leds >> 8) & 0xff);
}

int main(void) {
    char str1[] = "ELEC3042 BOPPIT!";
    char str2[] = "Address is 0x__ ";
    char str3[] = "LEVEL:__   BTN:_";
    char str4[] = "PRESS:          ";
    uint8_t LCD_Addr = 0x27;
    uint32_t    next;                   // when the next step ends
    uint32_t    step = 1000;            // delay step that is active
    uint16_t posn = 0b0000000000000001; // posn holds the LED that is on
    char    tone_on = 0;                // non zero when tone is on
    uint32_t    tone_off = 0;               // when to turn off the tone
    uint32_t    update_display = 0;     // by setting it to zero we update as soon as possible
    int level = 0;
    int oldLevel = -1;                  // start at -1 to ensure first update
    int press = 0;  // no pres
    char pressNone[]    = "     ";
    char pressHit[]     = "Hit  ";
    char pressMiss[]    = "Miss ";
    char pressClose[]   = "Close";
    int inPress = 0;                    // true if we are currently pressed
    
    setup();    // set up the physical hardware
    setupTimer1();
    setupTimer2();
    next = millis() + 1000;
    button_state = (PIND & 0b00000100)?Released:Pressed;
    setup_I2C();
    setup_SPI();
    
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
    LCD_Write(LCD_Addr, str1, 16);
    LCD_Position(LCD_Addr, 0x40);
    LCD_Write(LCD_Addr, str2, 16);
    
    LCD_Position(LCD_Addr, 0x4D);
    LCD_Write_Chr(LCD_Addr, ((LCD_Addr>>4)&0xf)+'0');
    LCD_Write_Chr(LCD_Addr, ((LCD_Addr>>0)&0xf)+'0');
    write_LEDs(posn);
    sei();
    next = millis() + 2000;
    while (millis() < next) {
        ;           // splash screen delay
    }
    LCD_Position(LCD_Addr, 0x00);
    LCD_Write(LCD_Addr, str3, 16);
    LCD_Position(LCD_Addr, 0x40);
    LCD_Write(LCD_Addr, str4, 16);
    while (1) {
        // State Machine 3: display updater
        if (millis() >= update_display) {
            /*
             * Update the lCD display. We update the level, the button state and
             * the last press state only if they have changed.
             */
            if (level != oldLevel) {
                oldLevel = level;
                LCD_Position(LCD_Addr, 0x06);
                LCD_Write_Chr(LCD_Addr, (level/10)+'0');
                LCD_Write_Chr(LCD_Addr, (level%10)+'0');
            }
            if (button_state != oldButton_state) {
                oldButton_state = button_state;
                LCD_Position(LCD_Addr, 0x0F);
                LCD_Write_Chr(LCD_Addr, (button_state == Pressed)?'P':'R');
            }
            if (press != oldPress) {
                oldPress = press;
                LCD_Position(LCD_Addr, 0x47);
                switch(press) {
                    case None:
                    default:
                        LCD_Write(LCD_Addr, pressNone, 5);
                        break;
                    case Hit:
                        LCD_Write(LCD_Addr, pressHit, 5);
                        break;
                    case Miss:
                        LCD_Write(LCD_Addr, pressMiss, 5);
                        break;
                    case Close:
                        LCD_Write(LCD_Addr, pressClose, 5);
                        break;
                }
            }
            update_display = millis() + 100;    // update in 100ms from now
        }
        // State Machine 2: check if it needs to do something
        if ((tone_on != 0) && (millis() >= tone_off)) {
            stopTones();
            tone_on = 0;
        }
        // State Machine 1: check if it needs to do something
        if (millis() >= next && !inPress) {
            if (buttonPressed()) {
                // button is pressed - check which LED is lit.
                button_state = Pressed;
                if (posn == 0b0000000000010000) { // middle green LED
                    press = Hit;
                    step >>= 1; // make it faster
                    if (step < 50) {
                        step = 50;    // fastest it can go.
                    }
                    level++;
                    startTone1();
                    tone_on = 1;
                    tone_off = millis() + 1000;
                    // green buzz
                } else if ((posn & 0b0000000000101000) == 0) {
                    press = Miss;
                    // not one of the orange LEDs, and not the green LED, so it
                    // must be a RED LED.
                    step = 1000;
                    level = 0;
                    startTone2();
                    tone_on = 1;
                    tone_off = millis() + 1000;
                    // red buzz
                } else {
                    press = Close;
                }
                inPress = 1;    // button is pressed - now we wait until it is released
            } else {
                button_state = Released;
                posn <<= 1;
                if (posn == 0b0000001000000000) {
                    posn = 0b0000000000000001;  // back to starting posn
                }
                write_LEDs(posn);   // update LEDs
            }
            next = millis() + step;     // when next to finish the loop
        } else if (!buttonPressed()) {
            inPress = 0;
        }
    }
}
