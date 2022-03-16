#include "avr/interrupt.h"
#include "avr/io.h"

#include "adc.h"
#include "display.h"
#include "fsm.h"
#include "timers.h"

bool displayPressed();
bool setPressed();
bool incrementPressed();
bool decrementPressed();

#define TIMER1_TICKS_PER_SECOND_1024 15625

#define DISPLAY_PORT PORTB
#define DISPLAY_DDR DDRB
#define DISPLAY_DATA_IN PB0

#define DISPLAY_CLOCK_PORT PORTD
#define DISPLAY_CLOCK_DDR DDRD
#define DISPLAY_SHIFT_PIN PD7
#define DISPLAY_LATCH_PIN PD4

inline void shiftBit()
{
    DISPLAY_CLOCK_PORT = (0 << DISPLAY_SHIFT_PIN);
    DISPLAY_CLOCK_PORT = (1 << DISPLAY_SHIFT_PIN);
}

inline void displayLatch()
{
    DISPLAY_CLOCK_PORT = (0 << DISPLAY_LATCH_PIN);
    DISPLAY_CLOCK_PORT = (1 << DISPLAY_LATCH_PIN);
}

void displaySendData(uint8_t data, DisplaySegment segIndex)
{
    uint16_t buffer = (data << 8) | segIndex;

    for (int i = 15; i >= 0; i--)
    {
        uint8_t bitToSend = (buffer >> i) & 1;
        DISPLAY_PORT = (DISPLAY_PORT & ~(1 << DISPLAY_DATA_IN)) |
                       (bitToSend << DISPLAY_DATA_IN);
        shiftBit();
    }
    displayLatch();
}

/**
 * @brief   Prepares Timer 1 for use as the primary time-keeping clock.
 *          NOTE: This does not start the timer, enableTimer(TC1, PRESCALER);
 * must be called to do so.
 *
 */
void initialiseTimer1()
{
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

void initialiseTimer0()
{
    TCCR0A = (1 << WGM01) | (0 << WGM00);
    TCCR0B = (0 << WGM02);
    TCNT0 = 0;
    OCR0A = 255;
    OCR0B = 0;
    TIMSK0 = (1 << OCIE0A);
    TIFR0 = 0x00;
}

volatile uint32_t secondsElasped = 0;

void initialiseADC()
{
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

void initialiseDisplay() {
    DISPLAY_DDR = (1 << DISPLAY_DATA_IN);
    DISPLAY_CLOCK_DDR = (1 << DISPLAY_SHIFT_PIN) | (1 << DISPLAY_LATCH_PIN);
}

int initialise()
{
    initialiseDisplay();
    initialiseButtons();
    initialiseTimer1();
    initialiseTimer0();
    initialiseADC();

    return 1;
}

void enableTimers()
{
    // Primary second counter
    enableTimer(TC1, CLOCK_SELECT_1024_PRESCALER);
    enableTimer(TC0, CLOCK_SELECT_1024_PRESCALER);
}

void displayUpdate(DisplayData displayData)
{
    for (int i = 0; i < 4; i++)
    {
        displaySendData(displayData.data[i], (1 << i));
    }
    // Clear all displays
    displaySendData(0xFF, 0x0F);
}

#define mapChar(c) SEGMENT_MAP[c]

volatile bool updateTimeDisplay = false;
volatile bool withDp = false;

typedef struct {
    uint8_t isPressed : 1;
    uint8_t hasBeenReleased : 1;
} Button;

#define BUTTON_SET PC1
#define BUTTON_SET_INT PCINT9
#define BUTTON_DISPLAY PC2
#define BUTTON_INCREMENT PC2
#define BUTTON_INCREMENT_INT PCINT10
#define BUTTON_DECREMENT PC3
#define BUTTON_DECREMENT_INT PCINT11
#define BUTTON_PORT PORTC
#define BUTTON_DDR DDRC

void initialiseButtons() {
    // Set buttons as inputs
    BUTTON_DDR &= ~((1 << BUTTON_SET) | (1 << BUTTON_INCREMENT) | (1 << BUTTON_DECREMENT));
    // Enable pull-up resistors for buttons
    BUTTON_PORT |= (1 << BUTTON_SET) | (1 << BUTTON_INCREMENT) | (1 << BUTTON_DECREMENT);

    // Enable interrupts for buttons
    PCMSK1 |= (1 << BUTTON_SET_INT) | (1 << BUTTON_INCREMENT_INT) | (1 << BUTTON_DECREMENT_INT);

    //Enable interrupt for button interrupt port
    PCICR |= (1 << PCIE1);
}

bool displayPressed()
{
}

bool setPressed()
{
}

bool incrementPressed()
{
}

bool decrementPressed()
{
}

void incrementHour()
{
}

void decrementHour()
{
}

void incrementMinute()
{
}

void decrementMinute()
{
}

FSM_TRANSITION displayHoursToDisplayMinutes = {DISPLAY_HH_MM, displayPressed, noAction, DISPLAY_MM_SS};
FSM_TRANSITION displayHoursToSetTime = {DISPLAY_HH_MM, setPressed, noAction, SET_TIME_MODE_HR};
FSM_TRANSITION displayMinutesToDisplayHours = {DISPLAY_MM_SS, displayPressed, noAction, DISPLAY_HH_MM};
FSM_TRANSITION timeSetHrToMin = {SET_TIME_MODE_HR, setPressed, noAction, SET_TIME_MODE_MIN};
FSM_TRANSITION timeSetHrIncrement = {SET_TIME_MODE_HR, incrementPressed, incrementHour, SET_TIME_MODE_HR};
FSM_TRANSITION timeSetHrDecrement = {SET_TIME_MODE_HR, decrementPressed, decrementHour, SET_TIME_MODE_HR};
FSM_TRANSITION timeSetMinToHr = {SET_TIME_MODE_MIN, setPressed, noAction, DISPLAY_HH_MM};
FSM_TRANSITION timeSetMinIncrement = {SET_TIME_MODE_MIN, incrementPressed, incrementMinute, SET_TIME_MODE_MIN};
FSM_TRANSITION timeSetMinDecrement = {SET_TIME_MODE_MIN, decrementPressed, decrementMinute, SET_TIME_MODE_MIN};

struct Time {
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
};

struct Time clock;

void addSecond(struct Time* clock) {
    clock->seconds++;

    if (clock->seconds == 60) {
        clock->minutes++;
        clock->seconds = 0;
    }
    if (clock->minutes == 60) {
        clock->hours++;
        clock->minutes = 0;
    }
}

int main()
{
    if (!initialise())
        return -1;

    enableTimers();

    sei();

    FSM_TRANSITION_TABLE stateMachine = {DISPLAY_HH_MM, {   displayHoursToDisplayMinutes,
                                                            displayHoursToSetTime,
                                                            displayMinutesToDisplayHours,
                                                            timeSetHrToMin,
                                                            timeSetHrIncrement,
                                                            timeSetHrDecrement,
                                                            timeSetMinToHr,
                                                            timeSetMinIncrement,
                                                            timeSetMinDecrement}};

    while (1)
    {
        FSMUpdate(&stateMachine);
    }

    return 0;
}

// Maybe put into a fsm.c file. This implementation is generic enough that we dont
// need it specified here.
void FSMUpdate(FSM_TRANSITION_TABLE *table)
{
    /**
     * Iterate overall potential transitions.
     * If any transition matches the current state, check its trigger(s).
     * If they have been fired, execute the transitions action and move to the next state.
     *
     */
    for (int index = 0; index < FSM_TRANSITION_MAX; index++)
    {
        FSM_TRANSITION transition = table->transitions[index];

        if (transition.currentState == table->currentState)
        {
            if (transition.trigger())
            {
                transition.action();
                table->currentState = transition.nextState;
                break;
            }
        }
    }
}

ISR(TIMER1_COMPA_vect)
{
    static uint8_t tick;
    tick++;

    withDp = (tick == 1);

    if (tick == 2)
    {
        //secondsElasped++;
        addSecond(&clock);
        tick = 0;
    }
}

volatile uint32_t adcValue = 0;
volatile DisplayData displayData = {{1, 2, 3, 4}};



ISR(TIMER0_COMPA_vect)
{
    // uint32_t secondsTemp = secondsElasped;
    updateTimeDisplay = true;
    uint32_t secondsTemp = secondsElasped;

    // displayData.data[3] = mapChar(secondsTemp & 0x0F);
    // displayData.data[2] = mapChar((secondsTemp >> 4) & 0x0F);
    // displayData.data[1] = mapChar((secondsTemp >> 8) & 0x0F);
    // displayData.data[0] = mapChar((secondsTemp >> 16) & 0x0F);

    uint8_t sl = clock.seconds % 10;
    uint8_t sh = clock.seconds / 10;

    uint8_t ml = clock.minutes % 10;
    uint8_t mh = clock.minutes / 10;

    displayData.data[3] = mapChar(sl);
    displayData.data[2] = mapChar(sh);
    displayData.data[1] = mapChar(ml);
    displayData.data[0] = mapChar(mh);

    // Decimal Point
    displayData.data[1] &= (withDp ? mapChar(DP) : 0xFF);

    displayUpdate(displayData);
}

#define ADC_LOWER_BUG_LIMIT 0x4

ISR(ADC_vect)
{
    // Set timer output compare to ADC value
    OCR0A = ADCH;

    if (OCR0A < ADC_LOWER_BUG_LIMIT)
    {
        OCR0A = ADC_LOWER_BUG_LIMIT;
    }

    adcValue = OCR0A;
}

/**
 * Button interrupt register
 * 
 */
ISR(PCINT1_vect) {
    
}