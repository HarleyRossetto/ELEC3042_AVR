#include "avr/interrupt.h"
#include "avr/io.h"

#include "adc.h"
#include "button.h"
#include "clock.h"
#include "fsm.h"
#include "sevenSegmentDisplay.h"
#include "systemtimer.h"
#include "tickable.h"
#include "timers.h"

#define NULL ((void *)0)

FSM_TRANSITION_TABLE *stateMachinePtr = 0;

/// TIMING
#define TIMER1_TICKS_PER_SECOND_256PRESCALE     F_CPU / 256
#define TIMER1_TICKS_PER_100_MILLIS_256PRESCALE TIMER1_TICKS_PER_SECOND_256PRESCALE / 10
#define TIMER1_PERIOD_MILLISECONDS              100

#define TIMER2_TICKS_PER_SECOND_64_PRESCALE     F_CPU / 64
#define TIMER2_TICKS_PER_MILLISEC_64_PRESCALE   TIMER2_TICKS_PER_SECOND_64_PRESCALE / 250
#define TIMER2_PERIOD_MILLISECONDS              1

void enableTimers() {
    // Primary millisecond counter
    enableTimer(TC2, TIMER2_CLOCK_SELECT_64_PRESCALER);
    // 7 Segment Display Update
    enableTimer(TC0, CLOCK_SELECT_1024_PRESCALER);
    // Buzzer
    // enableTimer(TC1, CLOCK_SELECT_256_PRESCALER);
}

volatile Time clock;

typedef enum { TWELVE_HOUR_TIME, TWENTY_FOUR_HOUR_TIME } TimeMode;

TimeMode timeMode = TWENTY_FOUR_HOUR_TIME;

/// TIMING

/// BUTTONS

bool setButtonPressed = false;
// Set Button pressed callback
void set_Pressed() { setButtonPressed = true; }

// Set Button released callback
bool setButtonHeld = false;
void set_Held() { setButtonHeld = true; }

bool incrementButtonPressed = false;
// Increment Button pressed callback
void inc_Pressed() { incrementButtonPressed = true; }

// Inc Button released callback
bool incrementButtonHeld = false;
void inc_Held() {
    incrementButtonHeld = true;
    PORTB ^= (1 << PB5);
}

bool decrementButtonPressed = false;
// Decrement Button pressed callback
void dec_Pressed() { decrementButtonPressed = true; }

bool decrementButtonHeld = false;
void dec_Held() { decrementButtonHeld = true; }

#define BUTTON_SET           PC1
#define BUTTON_SET_INT       PCINT9
#define BUTTON_DISPLAY       PC2
#define BUTTON_INCREMENT     PC2
#define BUTTON_INCREMENT_INT PCINT10
#define BUTTON_DECREMENT     PC3
#define BUTTON_DECREMENT_INT PCINT11
#define BUTTON_PORT          PORTC
#define BUTTON_DDR           DDRC
#define BUTTON_IN            PINC

bool displayPressed();
bool setPressed();
bool incrementPressed();
bool decrementPressed();

/// BUTTONS

/// DISPLAY

DisplayData displayData = {
    {0, 0, 0, 0}
};

typedef void (*DisplayFunctionPointer)();
DisplayFunctionPointer displayFunction;

#define DISPLAY_PORT       PORTB
#define DISPLAY_DDR        DDRB
#define DISPLAY_DATA_IN    PB0

#define DISPLAY_CLOCK_PORT PORTD
#define DISPLAY_CLOCK_DDR  DDRD
#define DISPLAY_SHIFT_PIN  PD7
#define DISPLAY_LATCH_PIN  PD4

#define mapChar(c)         SEGMENT_MAP[c]

volatile bool updateTimeDisplay = false;
volatile bool withDp            = true;

/// DISPLAY

/// INITIALISERS

/**
 * @brief   Prepares Timer 1 for use as the primary time-keeping clock.
 *          NOTE: This does not start the timer, enableTimer(TC1, PRESCALER);
 * must be called to do so.
 *
 * Timer 1 will run with a 100ms period using a 256 prescaler. With this
 * configuration no compensation is required because there are no fractions
 * involved in compare counts.
 */
void initialiseTimer1() {
    // Timer/Counter Control Register A (Compare Output Modes and Waveform
    // Generation Modes (bits 11 and 10))
    TCCR1A = (0 << WGM11) | (0 << WGM10); // Waveform Generation Mode - CTC

    // Timer starts once Clock Select is set, so we will configure that last.
    //  Timer/Counter Control Register B (Input Capture Noise Canceler, Input
    //  Capture Edge Select, Waveform Generation Mode (bits 13 and 12), and Clock
    //  Select)
    TCCR1B = (0 << WGM13) | (1 << WGM12);

    // Timer/Counter Control Register C (Force Output Compare)
    TCCR1C = 0x00;

    // Timer/Counter Value
    TCNT1 = 0;

    // Output Compare Register A
    OCR1A = TIMER1_TICKS_PER_100_MILLIS_256PRESCALE;
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
}

// Display Update Timer
void initialiseTimer0() {
    TCCR0A = (1 << WGM01) | (0 << WGM00);
    TCCR0B = (0 << WGM02);
    TCNT0  = 0;
    OCR0A  = 255;
    OCR0B  = 0;
    TIMSK0 = (1 << OCIE0A);
    TIFR0  = 0x00;
}

// Millisecond timer
void initialiseTimer2() {
    TCCR2A = (1 << WGM21) | (0 << WGM20);
    TCCR2B = (0 << WGM22);
    TCNT2  = 0;
    OCR2A  = 249; // Zero relative (0 - 249 = 250)
    OCR2B  = 0;
    TIMSK2 = (1 << OCF2A);
    TIFR2  = 0x00;
}

void initialiseADC() {
    // ADC Multiplexer Selection Register
    ADMUX = ADC_VREF_AVCC | ADC_CH_0 | (1 << ADLAR);

    // ADC Control and Status Register A
    // Enabled ADC, Enable Interrupts, Enable Auto Trigger
    ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADATE) | (1 << ADPS1) | (1 << ADPS0) | (1 << ADSC);

    // ADC Control and Status Register B
    // Sample based on timer0 compare A. NOT a great method because period can
    // change. Timer0 used for 7 segment brightness.
    ADCSRB = ADC_AUTO_TRIGGER_SOURCE_TCC0_COMPARE_MATCH_A;

    // ADC Data Register
    ADC = 0;

    // Digital Inout Disable Register
    DIDR0 = (1 << ADC0D); // Disable digital input on AD0
}

void toggleTimeDisplayModes() { timeMode = (timeMode == TWELVE_HOUR_TIME ? TWENTY_FOUR_HOUR_TIME : TWELVE_HOUR_TIME); }

void initialiseButtons() {
    ButtonCreate(&BUTTON_DDR, &BUTTON_PORT, &BUTTON_IN, BUTTON_SET, set_Pressed, set_Held, true, NULL);

    Tickable *tickableIncrement = TickableCreate(1000L, NULL, NULL, false, false);
    ButtonCreate(&BUTTON_DDR, &BUTTON_PORT, &BUTTON_IN, BUTTON_INCREMENT, inc_Pressed, inc_Held, true,
                 tickableIncrement);

    Tickable *tickableToggleDisplayModes = TickableCreate(2000L, NULL, NULL, false, true);
    ButtonCreate(&BUTTON_DDR, &BUTTON_PORT, &BUTTON_IN, BUTTON_DECREMENT, dec_Pressed, dec_Held, true,
                 tickableToggleDisplayModes);
}

int initialise() {
    SevenSegmentInitialise(&DISPLAY_DDR, DISPLAY_DATA_IN, &DISPLAY_PORT, &DISPLAY_CLOCK_DDR, &DISPLAY_CLOCK_PORT,
                           DISPLAY_SHIFT_PIN, DISPLAY_LATCH_PIN);
    initialiseButtons();
    initialiseTimer2();
    initialiseTimer1();
    initialiseTimer0();
    initialiseADC();

    // Initialise LEDS and set inital values.
    DDRB |= (1 << PB5) | (1 << PB4) | (1 << PB3) | (1 << PB2);
    PORTB |= (1 << PB5) | (1 << PB4) | (1 << PB3) | (1 << PB2);

    return 1;
}

/// INITIALISERS

bool buttonPressed(bool *buttonPressFlag) {
    bool result = *buttonPressFlag;
    if (result)
        *buttonPressFlag = false;

    return result;
}

bool buttonFlagActive(bool *flag) {
    bool result = *flag;
    if (result)
        *flag = false;

    return result;
}

bool displayPressed() { return decrementPressed(); }

bool displayHeld() { return buttonFlagActive(&decrementButtonHeld); }

bool setPressed() { return buttonPressed(&setButtonPressed); }

bool incrementPressed() { return buttonPressed(&incrementButtonPressed); }

bool incrementHeld() { return buttonFlagActive(&incrementButtonHeld); }

bool decrementPressed() { return buttonPressed(&decrementButtonPressed); }

bool incrementPressedOrHeld() { return incrementPressed() || incrementHeld(); }

void incrementHour() { addHours(&clock, 1); }

void decrementHour() { addHours(&clock, -1); }

void incrementMinute() { addMinutes(&clock, 1); }

void decrementMinute() { addMinutes(&clock, -1); }

void incrementSecond() { addSeconds(&clock, 1); }

void decrementSecond() { addSeconds(&clock, -1); }

/// Display Functions
bool transitioned = false;
uint8_t matches   = 0;
bool triggered    = false;

void displayFunctionCurrentState() {
    displayData.data[SEG_FAR_RIGHT] = mapChar(stateMachinePtr->currentState); // Far Right -> Current State Index
    displayData.data[SEG_RIGHT]     = mapChar(triggered ? 9 : 0); // Center Right -> 9 if triggered, 0 if not.
    displayData.data[SEG_LEFT]      = mapChar(matches);           // Centre Left -> FSM Matches for current state.
    displayData.data[SEG_FAR_LEFT]  = mapChar(transitioned);      // Far Left -> FSM has transitioned.
}

volatile uint32_t adcValue = 0;
void displayFunctionADCValue() {
    displayData.data[SEG_FAR_RIGHT] = mapChar(adcValue & 0x0F);
    displayData.data[SEG_RIGHT]     = mapChar((adcValue >> 4) & 0x0F);
    displayData.data[SEG_LEFT]      = mapChar((adcValue >> 8) & 0x0F);
    displayData.data[SEG_FAR_LEFT]  = mapChar((adcValue >> 12) & 0x0F);
}

void displayFunctionTimeHHMM() {
    uint8_t ml = clock.minutes % 10;
    uint8_t mh = clock.minutes / 10;

    uint8_t hours = clock.hours;

    // 12 hour time adjustment.
    if (timeMode == TWELVE_HOUR_TIME && hours > 12) {
        hours -= 12;
    }

    uint8_t hl = hours % 10;
    uint8_t hh = hours / 10;

    displayData.data[SEG_FAR_RIGHT] = mapChar(ml); // Far Right
    displayData.data[SEG_RIGHT]     = mapChar(mh); // Centre Right
    displayData.data[SEG_LEFT]      = mapChar(hl); // Center Left
    displayData.data[SEG_FAR_LEFT]  = mapChar(hh); // Far Left

    // Decimal Point
    displayData.data[SEG_LEFT] &= (withDp ? mapChar(DP) : mapChar(BLANK));

    // PM Indicator: Active if showing 12 hours time and hours are > 12.
    displayData.data[SEG_FAR_RIGHT] &=
        (timeMode == TWELVE_HOUR_TIME && clock.hours > 12 ? mapChar(DP) : mapChar(BLANK));
}

void displayFunctionTimeMMSS() {
    uint8_t sl = clock.seconds % 10;
    uint8_t sh = clock.seconds / 10;

    uint8_t ml = clock.minutes % 10;
    uint8_t mh = clock.minutes / 10;

    displayData.data[SEG_FAR_RIGHT] = mapChar(sl); // Far Right
    displayData.data[SEG_RIGHT]     = mapChar(sh); // Centre Right
    displayData.data[SEG_LEFT]      = mapChar(ml); // Center Left
    displayData.data[SEG_FAR_LEFT]  = mapChar(mh); // Far Left

    // Decimal Point
    // Must show constantly when in MMSS state. Currently does not.
    displayData.data[SEG_LEFT] &= mapChar(DP);

    // PM Indicator
    displayData.data[SEG_FAR_RIGHT] &=
        (timeMode == TWELVE_HOUR_TIME && clock.hours > 12 ? mapChar(DP) : mapChar(BLANK));
}

void displayFunctionSetTimeHH() {
    uint8_t hours = clock.hours;

    if (timeMode == TWELVE_HOUR_TIME && hours > 12) {
        hours -= 12;
    }

    uint8_t hl = hours % 10;
    uint8_t hh = hours / 10;

    displayData.data[SEG_FAR_RIGHT] = mapChar(BLANK); // Far Right
    displayData.data[SEG_RIGHT]     = mapChar(BLANK); // Centre Right
    displayData.data[SEG_LEFT]      = mapChar(hl);    // Center Left
    displayData.data[SEG_FAR_LEFT]  = mapChar(hh);    // Far Left

    // PM Indicator
    displayData.data[3] &= (timeMode == TWELVE_HOUR_TIME && clock.hours > 12 ? mapChar(DP) : mapChar(BLANK));
}

void displayFunctionTimeMM() {
    uint8_t ml = clock.minutes % 10;
    uint8_t mh = clock.minutes / 10;

    displayData.data[SEG_FAR_RIGHT] = mapChar(ml);    // Far Right
    displayData.data[SEG_RIGHT]     = mapChar(mh);    // Centre Right
    displayData.data[SEG_LEFT]      = mapChar(BLANK); // Center Left
    displayData.data[SEG_FAR_LEFT]  = mapChar(BLANK); // Far Left
}

void displayFunctionBlank() {
    displayData.data[SEG_FAR_RIGHT] = mapChar(BLANK); // Far Right
    displayData.data[SEG_RIGHT]     = mapChar(BLANK); // Centre Right
    displayData.data[SEG_LEFT]      = mapChar(BLANK); // Center Left
    displayData.data[SEG_FAR_LEFT]  = mapChar(BLANK); // Far Left
}

void displayFunctionIncHold() {
    Tickable *incTick = &GetButtons()[2].holdEvent;
    displayData.data[SEG_FAR_RIGHT] = mapChar((incTick->elaspedTime >> 4) & 0xF); // Far Right
    displayData.data[SEG_RIGHT]     = mapChar((incTick->elaspedTime >> 8) & 0xF); // Centre Right
    //displayData.data[SEG_FAR_RIGHT] = mapChar(BLANK);
    //displayData.data[SEG_RIGHT]     = mapChar(BLANK);
    displayData.data[SEG_LEFT]          = mapChar(incTick->oneShot & 1); // Center Left
    displayData.data[SEG_FAR_LEFT]      = mapChar(incTick->enabled & 1); // Far Left
}

volatile uint8_t isrTime    = 0;
volatile uint8_t isrTimeEnd = 0;
void displayFunctionISRTime() {
    // uint64_t delta = isrTimeEnd - isrTime;
    int8_t delta = isrTimeEnd - isrTime;

    displayData.data[SEG_FAR_RIGHT] = mapChar(delta & 0xF);         // Far Right
    displayData.data[SEG_RIGHT]     = mapChar((delta >> 4) & 0xF);  // Centre Right
    displayData.data[SEG_LEFT]      = mapChar((delta >> 8) & 0xF);  // Center Left
    displayData.data[SEG_FAR_LEFT]  = mapChar((delta >> 12) & 0xF); // Far Left
}

void displayFunctionTwo8BitReg(volatile uint8_t *r1, volatile uint8_t *r2) {
    displayData.data[SEG_FAR_RIGHT] = mapChar(*r2 & 0xF);        // Far Right
    displayData.data[SEG_RIGHT]     = mapChar((*r2 >> 4) & 0xF); // Centre Right
    displayData.data[SEG_LEFT]      = mapChar(*r1 & 0xF);        // Center Left
    displayData.data[SEG_FAR_LEFT]  = mapChar((*r1 >> 4) & 0xF); // Far Left
}

void displayFunction8BitReg(volatile uint8_t *r1) { displayFunctionTwo8BitReg(r1, 0); }

void displayFunctionPCICR() { displayFunction8BitReg(&PCICR); }

void displayFunctionDDRC() { displayFunctionTwo8BitReg(&DDRC, &PORTC); }

void displayFunctionButtonStates() {
    Button *btns                    = GetButtons();
    displayData.data[SEG_FAR_RIGHT] = mapChar(btns[2].currentState); // Far Right
    displayData.data[SEG_RIGHT]     = mapChar(btns[1].currentState); // Centre Right
    displayData.data[SEG_LEFT]      = mapChar(btns[0].currentState); // Center Left
    displayData.data[SEG_FAR_LEFT]  = mapChar(BLANK);                // Far Left
}

void resetSeconds() { clock.seconds = 0; }

/// Display Functions

/// Finite State Machine

FSM_TRANSITION displayHoursToDisplayMinutes = {
    DISPLAY_HH_MM,
    displayPressed,
    noAction,
    DISPLAY_MM_SS,
};
FSM_TRANSITION displayHoursToSetTime      = {DISPLAY_HH_MM, setPressed, noAction, SET_TIME_MODE_HR};
FSM_TRANSITION displayHoursToggleTimeMode = {DISPLAY_HH_MM, displayHeld, toggleTimeDisplayModes, DISPLAY_HH_MM};

FSM_TRANSITION displayMinutesToDisplayHours = {DISPLAY_MM_SS, displayPressed, noAction, DISPLAY_HH_MM};

FSM_TRANSITION timeSetHrToMin     = {SET_TIME_MODE_HR, setPressed, noAction, SET_TIME_MODE_MIN};
FSM_TRANSITION timeSetHrIncrement = {SET_TIME_MODE_HR, incrementPressedOrHeld, incrementHour, SET_TIME_MODE_HR};
FSM_TRANSITION timeSetHrDecrement = {SET_TIME_MODE_HR, decrementPressed, decrementHour, SET_TIME_MODE_HR};

FSM_TRANSITION timeSetMinToDisplayHr = {SET_TIME_MODE_MIN, setPressed, resetSeconds, DISPLAY_HH_MM};
FSM_TRANSITION timeSetMinIncrement   = {SET_TIME_MODE_MIN, incrementPressedOrHeld, incrementMinute, SET_TIME_MODE_MIN};
FSM_TRANSITION timeSetMinDecrement   = {SET_TIME_MODE_MIN, decrementPressed, decrementMinute, SET_TIME_MODE_MIN};

/// Finite State Machine

volatile bool buttonInterruptTriggered = false;

volatile bool shouldUpdateDisplay = false;

DisplayFunctionPointer displayFunctions[FSM_STATE_COUNT] = {displayFunctionTimeHHMM, displayFunctionTimeMMSS,
                                                            displayFunctionSetTimeHH, displayFunctionTimeMM};

void toggleDecimalPlaceDisplay() { withDp = !withDp; }

// #define OVERRIDE_DISPLAY_FUNCTION

Tickable *buzzerDisable = 0;

void disableBuzzer() { disableTimer(TC2); }

int main() {
    if (!initialise())
        return -1;

    addTime(&clock, 14, 43, 44);

    FSM_TRANSITION_TABLE stateMachine = {
        DISPLAY_HH_MM,
        {displayHoursToDisplayMinutes, displayHoursToggleTimeMode, displayHoursToSetTime, displayMinutesToDisplayHours,
          timeSetHrToMin, timeSetHrIncrement, timeSetHrDecrement, timeSetMinToDisplayHr, timeSetMinIncrement,
          timeSetMinDecrement}
    };
    stateMachinePtr = &stateMachine;

    // Create a tickable event which increments the seconds, once every second.
    TickableCreate(1000L, incrementSecond, 0, true, false);
    // Create a tickable even which toggle the decimal place display every 500ms.
    TickableCreate(500L, toggleDecimalPlaceDisplay, 0, true, false);
    // Create a tickable event for disabling the buzzer. Runs for 5 seconds,
    // disables buzzer and then is disabled (because it is created as a 1-shot
    // tickable).
    // buzzerDisable = TickableCreate(5000L, disableBuzzer, 0, false, true);

    enableTimers();

    sei();

#ifdef OVERRIDE_DISPLAY_FUNCTION
    displayFunction = displayFunctionIncHold;
#endif

    while (1) {
        // Clear all button inputs flags on major state change.
        if (FSMUpdate(&stateMachine) == STATE_CHANGE) {
            setButtonPressed       = false;
            incrementButtonPressed = false;
            decrementButtonPressed = false;

            setButtonHeld       = false;
            incrementButtonHeld = false;
            decrementButtonHeld = false;
        }

#ifndef OVERRIDE_DISPLAY_FUNCTION
        displayFunction = displayFunctions[stateMachine.currentState];
#endif

        if (shouldUpdateDisplay) {
            shouldUpdateDisplay = false;
            // Need to turn of display
            if (adcValue == 0xFF) {
                displayFunctionBlank();
            } else {
                if (displayFunction)
                    displayFunction();
            }

            SevenSegmentUpdate(displayData.data);
        }
    }

    return 0;
}

// Buzzer
ISR(TIMER1_COMPA_vect) {}

// Display update
ISR(TIMER0_COMPA_vect) { shouldUpdateDisplay = true; }

// Main Counter
ISR(TIMER2_COMPA_vect) {
    isrTime = TCNT2;
    // Add 1 millisecond to the system counter.
    addMillisToSystemCounter(TIMER2_PERIOD_MILLISECONDS);

    TickableUpdate(TIMER2_PERIOD_MILLISECONDS);

    isrTimeEnd = TCNT2;
}

ISR(ADC_vect) {
    // Set timer output compare to ADC value
    adcValue = OCR0A = ADCH;
}

// Button interrupt register
ISR(PCINT1_vect) {
    if (BUTTON_IN)
        ButtonUpdateAll();
}