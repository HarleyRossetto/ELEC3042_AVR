#include "avr/io.h"
#include "avr/interrupt.h"

#include "fsm.h"
#include "timers.h"
#include "display.h"

bool displayPressed();
bool setPressed();
bool incrementPressed();
bool decrementPressed();

#define TIMER1_TICKS_PER_SECOND_1024 15625
typedef uint8_t DisplaySegment;
#define DISPLAY_PORT PORTB
#define DISPLAY_DATA_IN PB0

#define SFTCLK PD7
#define LCHCLK PD4

inline void shiftBit()
{
    PORTD = (0 << SFTCLK);
    PORTD = (1 << SFTCLK);
}

inline void latchDisplay()
{
    PORTD = (0 << LCHCLK);
    PORTD = (1 << LCHCLK);
}

void sendData(uint8_t data, DisplaySegment segIndex)
{
    uint16_t buffer = (data << 8) | segIndex;

    for (int i = 15; i >= 0; i--)
    {
        uint8_t bitToSend = (buffer >> i) & 1;
        DISPLAY_PORT = (DISPLAY_PORT & ~(1 << DISPLAY_DATA_IN)) | (bitToSend << DISPLAY_DATA_IN);
        shiftBit();
    }
    latchDisplay();
}

void initialiseTimer1()
{
    // Timer/Counter Control Register A (Compare Output Modes and Waveform Generation Modes (bits 11 and 10))
    TCCR1A = (0 << COM1A1) | (0 << COM1A0) | // Compare Output Mode - Normal Port Operation
             (0 << WGM11) | (0 << WGM10);    // Waveform Generation Mode - CTC

    // Timer/Counter Control Register C (Force Output Compare)
    TCCR1C = 0x00;

    // Timer/Counter Value
    TCNT1 = 0;

    // Output Compare Register A
    OCR1A = TIMER1_TICKS_PER_SECOND_1024;
    // Output Compare Register B
    OCR1B = 0;

    // Input Capture Register
    ICR1 = 0x0;

    // Timer/Counter Interrupt Mask Register (Input Capture Interrupt Enable, Output Compare A/B Match Interrupt Enable, and Overflow Interrupt Enable)
    TIMSK1 = (1 << OCIE1A); // Output Compare A Interrupt Enable

    // Timer/Counter Interrupt Flag Register (Input Capture, Output Compare A/B and Overflow Flags)
    TIFR1 = 0x00; // Ensure all timer interrupt flags are cleared.

    // Timer starts once Clock Select is set, so we will configure that last.
    //  Timer/Counter Control Register B (Input Capture Noise Canceler, Input Capture Edge Select, Waveform Generation Mode (bits 13 and 12), and Clock Select)
    TCCR1B = (0 << WGM13) | (1 << WGM12);
    //    | // Waveform Generation Mode - CTC
    //(1 << CS12) | (0 << CS11) | (1 << CS10);   //Clock Select - 1024 prescaler
}

volatile uint32_t secondsElasped = 0;

int initialise()
{
    initialiseTimer1();

    DDRB = (1 << PB3) | (1 << PB0);
    DDRD = (1 << PD7) | (1 << PD4);
    return 1;
}

void enableTimers()
{
    // Primary second counter
    enableTimer(TC1, CLOCK_SELECT_1024_PRESCALER);
}

uint8_t digits[4];

void showDigits()
{
    for (int i = 0; i < 4; i++)
    {
        sendData(SEGMENT_MAP[digits[i]], (1 << i));
    }
}

int main(void)
{
    if (!initialise())
        return -1;

    enableTimers();

    sei();

    while (1)
    {
    }

    return 0;
}

ISR(TIMER1_COMPA_vect)
{
    secondsElasped++;

    PORTB ^= (1 << PB3);

    digits[3] = secondsElasped & 0xFF;
    digits[2] = (secondsElasped >> 8) & 0xFF;
    digits[1] = (secondsElasped >> 16) & 0xFF;
    digits[0] = (secondsElasped >> 24) & 0xFF;
    showDigits();
}