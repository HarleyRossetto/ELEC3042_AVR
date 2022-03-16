#include "avr/interrupt.h"
#include "avr/io.h"

#include "display.h"
#include "fsm.h"
#include "timers.h"

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

inline void shiftBit() {
  PORTD = (0 << SFTCLK);
  PORTD = (1 << SFTCLK);
}

inline void latchDisplay() {
  PORTD = (0 << LCHCLK);
  PORTD = (1 << LCHCLK);
}

void sendData(uint8_t data, DisplaySegment segIndex) {
  uint16_t buffer = (data << 8) | segIndex;

  for (int i = 15; i >= 0; i--) {
    uint8_t bitToSend = (buffer >> i) & 1;
    DISPLAY_PORT = (DISPLAY_PORT & ~(1 << DISPLAY_DATA_IN)) |
                   (bitToSend << DISPLAY_DATA_IN);
    shiftBit();
  }
  latchDisplay();
}

/**
 * @brief   Prepares Timer 1 for use as the primary time-keeping clock.
 *          NOTE: This does not start the timer, enableTimer(TC1, PRESCALER);
 * must be called to do so.
 *
 */
void initialiseTimer1() {
  // Timer/Counter Control Register A (Compare Output Modes and Waveform
  // Generation Modes (bits 11 and 10))
  TCCR1A = (0 << WGM11) | (0 << WGM10); // Waveform Generation Mode - CTC

  // Timer/Counter Control Register C (Force Output Compare)
  TCCR1C = 0x00;

  // Timer/Counter Value
  TCNT1 = 0;

  // Output Compare Register A
  OCR1A = TIMER1_TICKS_PER_SECOND_1024 / 2;
  // Output Compare Register B
  OCR1B = 0;

  // Input Capture Register
  ICR1 = 0x0;

  // Timer/Counter Interrupt Mask Register (Input Capture Interrupt Enable,
  // Output Compare A/B Match Interrupt Enable, and Overflow Interrupt Enable)
  TIMSK1 = (1 << OCIE1A); // Output Compare A & B Interrupt Enable

  // Timer/Counter Interrupt Flag Register (Input Capture, Output Compare A/B
  // and Overflow Flags)
  TIFR1 = 0x00; // Ensure all timer interrupt flags are cleared.

  // Timer starts once Clock Select is set, so we will configure that last.
  //  Timer/Counter Control Register B (Input Capture Noise Canceler, Input
  //  Capture Edge Select, Waveform Generation Mode (bits 13 and 12), and Clock
  //  Select)
  TCCR1B = (0 << WGM13) | (1 << WGM12);
  //    | // Waveform Generation Mode - CTC
  //(1 << CS12) | (0 << CS11) | (1 << CS10);   //Clock Select - 1024 prescaler
}

void initialiseTimer0() {
  TCCR0A = (1 << WGM01) | (0 << WGM00);
  TCCR0B = (0 << WGM02);
  TCNT0 = 0;
  OCR0A = 255;
  OCR0B = 0;
  TIMSK0 = (1 << OCIE0A);
  TIFR0 = 0x00;
}

volatile uint32_t secondsElasped = 0;

#define ADC_VREF_OFF (0 << REFS1) | (0 << REFS0)
#define ADC_VREF_AVCC (0 << REFS1) | (1 << REFS0)
#define ADC_VREF_AREF (1 << REFS1) | (1 << REFS0)

#define ADC_CH_0 0b0000
#define ADC_CH_1 0b0001
#define ADC_CH_2 0b0010
#define ADC_CH_3 0b0011
#define ADC_CH_4 0b0100
#define ADC_CH_5 0b0101
#define ADC_CH_6 0b0110
#define ADC_CH_7 0b0111
#define ADC_CH_8 0b1000

void initialiseADC() {
  // ADC Multiplexer Selection Register
  ADMUX = ADC_VREF_AVCC | ADC_CH_0 | (1 << ADLAR);

  // ADC Control and Status Register A
  // Enabled ADC, Enable Interrupts, Enable Auto Trigger
  ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADATE) | (1 << ADPS1) |
           (1 << ADPS0) | (1 << ADSC);

  // ADC Control and Status Register B
  // Sample based on timer0 compare A. NOT a great method because period can
  // change. Timer0 used for 7 segment brightness.
  ADCSRB = (0 << ADTS2) | (1 << ADTS1) | (1 << ADTS0);

  // ADC Data Register
  ADC = 0;

  // Digital Inout Disable Register
  DIDR0 = (1 << ADC0D); // Disable digital input on AD0
}

int initialise() {
  initialiseTimer1();
  initialiseTimer0();
  initialiseADC();

  DDRB = (1 << PB3) | (1 << PB0);
  PORTB = (1 << PB3); // D3 tick LED
  DDRD = (1 << PD7) | (1 << PD4);
  return 1;
}

void enableTimers() {
  // Primary second counter
  enableTimer(TC1, CLOCK_SELECT_1024_PRESCALER);
  enableTimer(TC0, CLOCK_SELECT_1024_PRESCALER);
}

uint8_t digits[4];

void showDigits() {
  for (int i = 0; i < 4; i++) {
    sendData(digits[i], (1 << i));
  }
  // Clear all displays
  sendData(0xFF, 0x0F);
}

#define mapChar(c) SEGMENT_MAP[c]

volatile bool updateTimeDisplay = false;
volatile bool withDp = false;
int main(void) {
  if (!initialise())
    return -1;

  enableTimers();

  sei();

  while (1) {
    // if (updateTimeDisplay)
    // {
    //     uint32_t secondsTemp = secondsElasped;

    //     digits[3] = mapChar(secondsTemp & 0xFF);
    //     digits[2] = mapChar((secondsTemp >> 8) & 0xFF);
    //     digits[1] = mapChar((secondsTemp >> 16) & 0xFF);
    //     digits[0] = mapChar((secondsTemp >> 24) & 0xFF);
    //     //Decimal Point
    //     digits[1] &= (withDp ? mapChar(DP) : 0xFF);

    //     showDigits();
    //     updateTimeDisplay = false;
    // }
  }

  return 0;
}

ISR(TIMER1_COMPA_vect) {
  static uint8_t tick;
  tick++;
  PORTB ^= (1 << PB3);

  withDp = (tick == 1);

  if (tick == 2) {
    secondsElasped++;
    tick = 0;
  }
}

volatile uint32_t adcValue = 0;

ISR(TIMER0_COMPA_vect) {
  // uint32_t secondsTemp = secondsElasped;
  updateTimeDisplay = true;
  uint32_t secondsTemp = secondsElasped;

  digits[3] = mapChar(secondsTemp & 0x0F);
  digits[2] = mapChar((secondsTemp >> 4) & 0x0F);
  digits[1] = mapChar((secondsTemp >> 8) & 0x0F);
  digits[0] = mapChar((secondsTemp >> 16) & 0x0F);
  // Decimal Point
  digits[1] &= (withDp ? mapChar(DP) : 0xFF);

  showDigits();
}

#define ADC_LOWER_BUG_LIMIT 0x4

ISR(ADC_vect) {
  // Set timer output compare to ADC value
  OCR0A = ADCH;

  if (OCR0A < ADC_LOWER_BUG_LIMIT) {
    OCR0A = ADC_LOWER_BUG_LIMIT;
  }

  adcValue = OCR0A;
}