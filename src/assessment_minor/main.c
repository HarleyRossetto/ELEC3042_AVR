#include "avr/interrupt.h"
#include "avr/io.h"

#include "adc.h"
#include "display.h"
#include "fsm.h"
#include "timers.h"
#include "button.h"
#include "clock.h"
#include "systemtimer.h"

FSM_TRANSITION_TABLE *stateMachinePtr = 0;

/// TIMING

// volatile uint64_t currentTimeMilliseconds = 0;
// volatile uint16_t millisAccountedBetweenTicks;
// #define addMillisecondsToSystemCounter(millis) currentTimeMilliseconds += millis;

#define TIMER1_TICKS_PER_SECOND_256 62500
#define TIMER1_TICKS_PER_100_MILLIS_256PRESCALE TIMER1_TICKS_PER_SECOND_256 / 10

void enableTimers()
{
    // Primary second counter
    enableTimer(TC1, CLOCK_SELECT_256_PRESCALER);
    enableTimer(TC0, CLOCK_SELECT_1024_PRESCALER);
}

volatile Time clock;

typedef enum
{
    TWELVE_HOUR_TIME,
    TWENTY_FOUR_HOUR_TIME
} TimeMode;

TimeMode timeMode = TWELVE_HOUR_TIME;

/// TIMING

/// BUTTONS

bool buttonHasUpdate(volatile Button *btn) {
    bool result = btn->updated;
    btn->updated = false;
    return result;
}

bool buttonHasNewPress(volatile Button *btn) {
    return buttonHasUpdate(btn) && btn->currentState == PRESSED;
}

bool setButtonPressed = false;
// Set Button pressed callback
void set_Pressed() {
    PORTB &= ~(1 << PB5);
    setButtonPressed = true;
}

//Set Button released callback
void set_Released() {
    PORTB |= (1 << PB5);
}

bool incrementButtonPressed = false;
// Increment Button pressed callback
void inc_Pressed() {
    PORTB &= ~(1 << PB4);
    incrementButtonPressed = true;
}

//Inc Button released callback
void inc_Released() {
    PORTB |= (1 << PB4);
}

bool decrementButtonPressed = false;
// Decrement Button pressed callback
void dec_Pressed()
{
    PORTB &= ~(1 << PB3);
    decrementButtonPressed = true;
}

//Decrement Button released callback
void dec_Released() {
    PORTB |= (1 << PB3);
}


#define BUTTON_SET PC1
#define BUTTON_SET_INT PCINT9
#define BUTTON_DISPLAY PC2
#define BUTTON_INCREMENT PC2
#define BUTTON_INCREMENT_INT PCINT10
#define BUTTON_DECREMENT PC3
#define BUTTON_DECREMENT_INT PCINT11
#define BUTTON_PORT PORTC
#define BUTTON_DDR DDRC
#define BUTTON_IN PINC

volatile Button buttonSet = {RELEASED, 0, BUTTON_SET,       set_Pressed, set_Released, false, &BUTTON_IN};
volatile Button buttonInc = {RELEASED, 0, BUTTON_INCREMENT, inc_Pressed, inc_Released, false, &BUTTON_IN};
volatile Button buttonDec = {RELEASED, 0, BUTTON_DECREMENT, dec_Pressed, dec_Released, false, &BUTTON_IN};

#define BUTTON_COUNT 3
volatile Button buttons[BUTTON_COUNT];

bool displayPressed();
bool setPressed();
bool incrementPressed();
bool decrementPressed();

/// BUTTONS

/// DISPLAY

volatile DisplayData displayData = {{1, 2, 3, 4}};

typedef void (*DisplayFunctionPointer)();
DisplayFunctionPointer displayFunction;

#define DISPLAY_PORT PORTB
#define DISPLAY_DDR DDRB
#define DISPLAY_DATA_IN PB0

#define DISPLAY_CLOCK_PORT PORTD
#define DISPLAY_CLOCK_DDR DDRD
#define DISPLAY_SHIFT_PIN PD7
#define DISPLAY_LATCH_PIN PD4

inline void displayShiftBit()
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
        displayShiftBit();
    }
    displayLatch();
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

/// DISPLAY


/// INITIALISERS

/**
 * @brief   Prepares Timer 1 for use as the primary time-keeping clock.
 *          NOTE: This does not start the timer, enableTimer(TC1, PRESCALER);
 * must be called to do so.
 *
 * Timer 1 will run with a 100ms period using a 256 prescaler. With this configuration no 
 * compensation is required because there are no fractions involved in compare counts.
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

#define ADC_AUTO_TRIGGER_SOURCE_TCC0_COMPARE_MATCH_A ((0 << ADTS2) | (1 << ADTS1) | (1 << ADTS0))

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
    ADCSRB = ADC_AUTO_TRIGGER_SOURCE_TCC0_COMPARE_MATCH_A;

    // ADC Data Register
    ADC = 0;

    // Digital Inout Disable Register
    DIDR0 = (1 << ADC0D); // Disable digital input on AD0
}

void initialiseDisplay() {
    DISPLAY_DDR = (1 << DISPLAY_DATA_IN);
    DISPLAY_CLOCK_DDR = (1 << DISPLAY_SHIFT_PIN) | (1 << DISPLAY_LATCH_PIN);
}

void initialiseButtons() {
    // Set buttons as inputs
    BUTTON_DDR &= ~((1 << BUTTON_SET) | (1 << BUTTON_INCREMENT) | (1 << BUTTON_DECREMENT));
    // Enable pull-up resistors for buttons
    BUTTON_PORT |= (1 << BUTTON_SET) | (1 << BUTTON_INCREMENT) | (1 << BUTTON_DECREMENT);

    // Enable interrupts for buttons
    PCMSK1 |= (1 << BUTTON_SET_INT) | (1 << BUTTON_INCREMENT_INT) | (1 << BUTTON_DECREMENT_INT);

    //Enable interrupt for button interrupt port
    PCICR |= (1 << PCIE1);

    buttons[0] = buttonSet;
    buttons[1] = buttonInc;
    buttons[2] = buttonDec;

    setTiming(&TCNT1, TIMER1_TICKS_PER_100_MILLIS_256PRESCALE);
}

int initialise()
{
    initialiseDisplay();
    initialiseButtons();
    initialiseTimer1();
    initialiseTimer0();
    initialiseADC();

    // Initialise LEDS and set inital values.
    DDRB  |= (1 << PB5) | (1 << PB4) | (1 << PB3) | (1 << PB2);
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

bool displayPressed()
{
    return decrementPressed();
}

bool setPressed()
{
    // return buttonHasNewPress(&buttonSet);
    return buttonPressed(&setButtonPressed);
}

bool incrementPressed()
{
    // return buttonHasNewPress(&buttonInc);
    return buttonPressed(&incrementButtonPressed);
}

bool decrementPressed()
{
    // return buttonHasNewPress(&buttonDec);
    return buttonPressed(&decrementButtonPressed);
}

void incrementHour()
{
    addHours(&clock, 1);
}

void decrementHour()
{
    addHours(&clock, -1);
}

void incrementMinute()
{
    addMinutes(&clock, 1);
}

void decrementMinute()
{
    addMinutes(&clock, -1);
}

/// Display Functions
bool transitioned = false;
uint8_t matches = 0;
bool triggered = false;

void displayFunctionCurrentState()
{
    displayData.data[3] = mapChar(stateMachinePtr->currentState);    //Far Right -> Current State Index
    displayData.data[2] = mapChar(triggered ? 9 : 0); //Center Right -> 9 if triggered, 0 if not.
    displayData.data[1] = mapChar(matches);         //Centre Left -> FSM Matches for current state.
    displayData.data[0] = mapChar(transitioned);    //Far Left -> FSM has transitioned.
}

volatile uint32_t adcValue = 0;

void displayFunctionADCValue() {
    displayData.data[3] = mapChar(adcValue & 0x0F);
    displayData.data[2] = mapChar((adcValue >> 4) & 0x0F);
    displayData.data[1] = mapChar((adcValue >> 8) & 0x0F);
    displayData.data[0] = mapChar((adcValue >> 12) & 0x0F);
}

void displayFunctionTimeHHMM()
{
    uint8_t ml = clock.minutes % 10;
    uint8_t mh = clock.minutes / 10;

    uint8_t hours = clock.hours;

    if (timeMode == TWELVE_HOUR_TIME && hours > 12) {
        hours -= 12;
    }

    uint8_t hl = hours % 10;
    uint8_t hh = hours / 10;

    displayData.data[3] = mapChar(ml);  //Far Right
    displayData.data[2] = mapChar(mh);  //Centre Right
    displayData.data[1] = mapChar(hl);  //Center Left
    displayData.data[0] = mapChar(hh);  //Far Left

    // Decimal Point
    //Must show constantly when in MMSS state. Currently does not.
    displayData.data[1] &= (withDp ? mapChar(DP) : mapChar(BLANK));

    //PM Indicator
    displayData.data[3] &= (timeMode == TWELVE_HOUR_TIME && clock.hours > 12 ? mapChar(DP) : mapChar(BLANK));
}

void displayFunctionTimeMMSS()
{
    uint8_t sl = clock.seconds % 10;
    uint8_t sh = clock.seconds / 10;

    uint8_t ml = clock.minutes % 10;
    uint8_t mh = clock.minutes / 10;

    displayData.data[3] = mapChar(sl);  //Far Right
    displayData.data[2] = mapChar(sh);  //Centre Right
    displayData.data[1] = mapChar(ml);  //Center Left
    displayData.data[0] = mapChar(mh);  //Far Left

    // Decimal Point
    //Must show constantly when in MMSS state. Currently does not.
    displayData.data[1] &= mapChar(DP);

    // PM Indicator
    displayData.data[3] &= (timeMode == TWELVE_HOUR_TIME && clock.hours > 12 ? mapChar(DP) : mapChar(BLANK));
}

void displayFunctionSetTimeHH()
{
    uint8_t hours = clock.hours;

    if (timeMode == TWELVE_HOUR_TIME && hours > 12) {
        hours -= 12;
    }

    uint8_t hl = hours % 10;
    uint8_t hh = hours / 10;

    displayData.data[3] = mapChar(BLANK);   //Far Right
    displayData.data[2] = mapChar(BLANK);   //Centre Right
    displayData.data[1] = mapChar(hl);      //Center Left
    displayData.data[0] = mapChar(hh);      //Far Left

    //PM Indicator
    displayData.data[3] &= (timeMode == TWELVE_HOUR_TIME && clock.hours > 12 ? mapChar(DP) : mapChar(BLANK));
}

void displayFunctionTimeMM()
{
    uint8_t ml = clock.minutes % 10;
    uint8_t mh = clock.minutes / 10;

    displayData.data[3] = mapChar(ml);  //Far Right
    displayData.data[2] = mapChar(mh);  //Centre Right
    displayData.data[1] = mapChar(BLANK);  //Center Left
    displayData.data[0] = mapChar(BLANK);  //Far Left
}

void displayFunctionBlank() {
    displayData.data[3] = mapChar(BLANK);   //Far Right
    displayData.data[2] = mapChar(BLANK);   //Centre Right
    displayData.data[1] = mapChar(BLANK);      //Center Left
    displayData.data[0] = mapChar(BLANK);      //Far Left
}

void resetSeconds() {
    clock.seconds = 0;
}

/// Display Functions

/// Finite State Machine

/*
    BUTTON/CHANGE STATE ERROR

    i.e. when in MMSS mode, pressing SET raises SETPRESSED flag which isnt checked until return to HHMM state.
    Will need to clear flag after x time.
*/

FSM_TRANSITION displayHoursToDisplayMinutes = {DISPLAY_HH_MM,       displayPressed,     noAction,           DISPLAY_MM_SS,    };
FSM_TRANSITION displayHoursToSetTime        = {DISPLAY_HH_MM,       setPressed,         noAction,           SET_TIME_MODE_HR  };
FSM_TRANSITION displayMinutesToDisplayHours = {DISPLAY_MM_SS,       displayPressed,     noAction,           DISPLAY_HH_MM     };
FSM_TRANSITION timeSetHrToMin               = {SET_TIME_MODE_HR,    setPressed,         noAction,           SET_TIME_MODE_MIN };
FSM_TRANSITION timeSetHrIncrement           = {SET_TIME_MODE_HR,    incrementPressed,   incrementHour,      SET_TIME_MODE_HR  };
FSM_TRANSITION timeSetHrDecrement           = {SET_TIME_MODE_HR,    decrementPressed,   decrementHour,      SET_TIME_MODE_HR  };
FSM_TRANSITION timeSetMinToDisplayHr        = {SET_TIME_MODE_MIN,   setPressed,         resetSeconds,       DISPLAY_HH_MM     };
FSM_TRANSITION timeSetMinIncrement          = {SET_TIME_MODE_MIN,   incrementPressed,   incrementMinute,    SET_TIME_MODE_MIN };
FSM_TRANSITION timeSetMinDecrement          = {SET_TIME_MODE_MIN,   decrementPressed,   decrementMinute,    SET_TIME_MODE_MIN };

/// Finite State Machine


volatile bool buttonInterruptTriggered = false;

void updateAllButtons() {
    for (int i = 0; i < BUTTON_COUNT; i++) {
        updateButton(&buttons[i]);
    }
}

void fsmTransitionCallback(TransitionCallback result) {
    switch (result.reason) {
        case TRANSITIONED:
            transitioned = true;
            break;
        case NO_MATCH: 
            matches = result.data;
            break;
        case MATCHES:
            matches = result.data;
            break;
        case NOT_TRIGGERED:
            triggered = true;
            break;
        case TRIGGERED:
            break;
        case ACTION:
            break;
        }
}

volatile bool shouldUpdateDisplay = false;

DisplayFunctionPointer displayFunctions[FSM_STATE_COUNT] = {displayFunctionTimeHHMM, displayFunctionTimeMMSS, displayFunctionSetTimeHH, displayFunctionTimeMM};

// #define OVERRIDE_DISPLAY_FUNCTION

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
                                                            timeSetMinToDisplayHr,
                                                            timeSetMinIncrement,
                                                            timeSetMinDecrement}};
    stateMachinePtr = &stateMachine;

    #ifdef OVERRIDE_DISPLAY_FUNCTION
        displayFunction = displayFunctionTimeHHMM;
        // displayFunction = displayFunctionADCValue;
    #endif

    while (1)
    {
        //Clear all button inputs flags on major state change.
        if (FSMUpdate(&stateMachine) == STATE_CHANGE) {
            setButtonPressed = false;
            incrementButtonPressed = false;
            decrementButtonPressed = false;
        }

        #ifndef OVERRIDE_DISPLAY_FUNCTION
            displayFunction = displayFunctions[stateMachine.currentState];
        #endif

        if (shouldUpdateDisplay) {
            shouldUpdateDisplay = false;
            //Need to turn of display
            if (adcValue == 0xFF) {
                displayFunctionBlank();
            } else {
                if (displayFunction)
                   displayFunction();
            }

            displayUpdate(displayData);
        }
    }

    return 0;
}

ISR(TIMER1_COMPA_vect)
{
    static uint8_t tick;

    withDp = (tick > 5);

    tick++;


    if (tick > 9) {
        //secondsElasped++;
        addSeconds(&clock, 1);
        tick = 0;
    }
    //Add 100 milliseconds - any millis already accounted for previously to the system counter.
    addMillisToSystemCounter(100 - millisecondsHandledBetweenTicks);
}

ISR(TIMER0_COMPA_vect)
{
    shouldUpdateDisplay = true;
}

ISR(ADC_vect)
{
    // Set timer output compare to ADC value
    adcValue = OCR0A = ADCH;
}

/**
 * Button interrupt register
 * 
 */
ISR(PCINT1_vect) {
    if (BUTTON_IN)
         updateAllButtons();
}