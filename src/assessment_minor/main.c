#include "avr/interrupt.h"
#include "avr/io.h"

#include "adc.h"
#include "display.h"
#include "fsm.h"
#include "timers.h"

/// TIMING

volatile uint64_t currentTimeMilliseconds = 0;
volatile uint16_t millisAccountedBetweenTicks;
#define addMillisecondsToSystemCounter(millis) currentTimeMilliseconds += millis;

#define TIMER1_TICKS_PER_SECOND_1024 15625

void enableTimers()
{
    // Primary second counter
    enableTimer(TC1, CLOCK_SELECT_1024_PRESCALER);
    enableTimer(TC0, CLOCK_SELECT_1024_PRESCALER);
}

struct Time {
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
};

struct Time clock;
typedef enum
{
    TWELVE_HOUR_TIME,
    TWENTY_FOUR_HOUR_TIME
} TimeMode;

TimeMode timeMode = TWELVE_HOUR_TIME;

typedef enum
{
    HHMM,
    MMSS
} TimeDisplayMode;

TimeDisplayMode timeDisplayMode = HHMM;

void validateClock(struct Time *clock) {
    if (clock->seconds == 60) {
        clock->minutes++;
        clock->seconds = 0;
    }
    if (clock->minutes == 60) {
        clock->hours++;
        clock->minutes = 0;
    }
    if (clock->hours == 24) {
        clock->hours = 0;
        clock->minutes = 0;
        clock->seconds = 0;
    }
}

void addSecond(struct Time* clock) {
    clock->seconds++;

    validateClock(clock);
}

void addMinute(struct Time* clock) {
    clock->minutes++;
    validateClock(clock);
}

void removeMinute(struct Time *clock) {
    clock->minutes--;
    validateClock(clock);
}

void addHour(struct Time* clock) {
    clock->hours++;
    validateClock(clock);
}

void removeHour(struct Time *clock) {
    clock->hours--;
    validateClock(clock);
}

/// TIMING


/// BUTTONS

typedef enum
{
    NONE, PRESSED, RELEASED, HELD
} ButtonState;

#define BUTTON_CHANGE_DELAY_MS 15

typedef struct {
    ButtonState currentState;
    ButtonState actionState;
    uint64_t lastActionTime;
    uint8_t buttonPin;
    void(*eventPress)();
    void(*eventRelease)();
    bool updated;
} Button;

void buttonPress(Button* btn) {
    btn->currentState = PRESSED;
    btn->actionState = PRESSED;
    btn->updated = true;
}

void buttonRelease(Button* btn) {
    btn->currentState = RELEASED;
    btn->actionState = RELEASED;
    btn->updated = true;
}

ButtonState buttonReadActionState(Button *btn) {
    ButtonState result = btn->actionState;
    btn->actionState = NONE;
    return result;
}

bool buttonHasUpdate(Button *btn) {
    bool result = btn->updated;
    btn->updated = false;
    return result;
}

bool buttonHasNewPress(Button *btn) {
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

volatile Button buttonSet = {RELEASED, NONE, 0, BUTTON_SET, set_Pressed, set_Released, false};
volatile Button buttonInc = {RELEASED, NONE, 0, BUTTON_INCREMENT, inc_Pressed, inc_Released, false};
volatile Button buttonDec = {RELEASED, NONE, 0, BUTTON_DECREMENT, dec_Pressed, dec_Released, false};

#define BUTTON_COUNT 3
volatile Button buttons[BUTTON_COUNT];

bool displayPressed();
bool setPressed();
bool incrementPressed();
bool decrementPressed();

#define getButtonState(buttonPin, buttonInput) (buttonInput & (1 << buttonPin) ? RELEASED : PRESSED)
#define isPressed(buttonPin, buttonInput) !(buttonInput & (1 << buttonPin))
/// BUTTONS

/// DISPLAY

volatile DisplayData displayData = {{1, 2, 3, 4}};

void (*displaySetData)();

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
}

int initialise()
{
    initialiseDisplay();
    initialiseButtons();
    initialiseTimer1();
    initialiseTimer0();
    initialiseADC();

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
    bool result = decrementPressed();
    if (result) {
        PORTB &= ~(1 << PB2);
    }
    return result;
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
    addHour(&clock);
}

void decrementHour()
{
    removeHour(&clock);
}

void incrementMinute()
{
    addMinute(&clock);
}

void decrementMinute()
{
    removeMinute(&clock);
}

void displayHHMMAction() {
    timeDisplayMode = HHMM;
}

void displayMMSSAction() {
    timeDisplayMode = MMSS;
}

/// Finite State Machine

FSM_TRANSITION displayHoursToDisplayMinutes = {DISPLAY_HH_MM,       displayPressed,     displayMMSSAction, DISPLAY_MM_SS};
FSM_TRANSITION displayHoursToSetTime        = {DISPLAY_HH_MM,       setPressed,         noAction, SET_TIME_MODE_HR};
FSM_TRANSITION displayMinutesToDisplayHours = {DISPLAY_MM_SS,       displayPressed,     displayHHMMAction, DISPLAY_HH_MM};
FSM_TRANSITION timeSetHrToMin               = {SET_TIME_MODE_HR,    setPressed,         noAction, SET_TIME_MODE_MIN};
FSM_TRANSITION timeSetHrIncrement           = {SET_TIME_MODE_HR,    incrementPressed,   incrementHour, SET_TIME_MODE_HR};
FSM_TRANSITION timeSetHrDecrement           = {SET_TIME_MODE_HR,    decrementPressed,   decrementHour, SET_TIME_MODE_HR};
FSM_TRANSITION timeSetMinToHr               = {SET_TIME_MODE_MIN,   setPressed,         noAction, DISPLAY_HH_MM};
FSM_TRANSITION timeSetMinIncrement          = {SET_TIME_MODE_MIN,   incrementPressed,   incrementMinute, SET_TIME_MODE_MIN};
FSM_TRANSITION timeSetMinDecrement          = {SET_TIME_MODE_MIN,   decrementPressed,   decrementMinute, SET_TIME_MODE_MIN};

/// Finite State Machine

uint8_t currentState = 0;

bool transitioned = false;
uint8_t matches = 0;
bool triggered = false;
void displayCurrentState()
{
    displayData.data[3] = mapChar(currentState);    //Far Right -> Current State Index
    displayData.data[2] = mapChar(triggered ? 9 : 0); //Center Right -> 9 if triggered, 0 if not.
    displayData.data[1] = mapChar(matches);         //Centre Left -> FSM Matches for current state.
    displayData.data[0] = mapChar(transitioned);    //Far Left -> FSM has transitioned.
}


void displayTime()
{
    uint8_t sl = clock.seconds % 10;
    uint8_t sh = clock.seconds / 10;

    uint8_t ml = clock.minutes % 10;
    uint8_t mh = clock.minutes / 10;

    uint8_t hours = clock.hours;

    if (timeMode == TWELVE_HOUR_TIME && hours > 12) {
        hours -= 12;
    }

    uint8_t hl = hours % 10;
    uint8_t hh = hours / 10;


    if (timeDisplayMode == HHMM) {
        displayData.data[3] = mapChar(ml);  //Far Right
        displayData.data[2] = mapChar(mh);  //Centre Right
        displayData.data[1] = mapChar(hl);  //Center Left
        displayData.data[0] = mapChar(hh);  //Far Left
    } else {
        displayData.data[3] = mapChar(sl);  //Far Right
        displayData.data[2] = mapChar(sh);  //Centre Right
        displayData.data[1] = mapChar(ml);  //Center Left
        displayData.data[0] = mapChar(mh);  //Far Left
    }

    // Decimal Point
    displayData.data[1] &= (withDp ? mapChar(DP) : mapChar(BLANK));

    //PM Indicator
    displayData.data[3] &= (timeMode == TWELVE_HOUR_TIME && clock.hours > 12 ? mapChar(DP) : mapChar(BLANK));
}

volatile bool buttonInterruptTriggered = false;

bool buttonTimingCorrect(Button *btn) {
    return currentTimeMilliseconds - btn->lastActionTime > BUTTON_CHANGE_DELAY_MS;
}

void updateButton(Button* btn) {

    // If the button was last considered released and is now pressed.
    if (btn->currentState == RELEASED && isPressed(btn->buttonPin, BUTTON_IN) && buttonTimingCorrect(btn))
    {
         //Calculate milliseconds elasped since last clock call.
        uint16_t  millisElaspedAtInterruptCall = (TCNT1 * 500 / (TIMER1_TICKS_PER_SECOND_1024 / 2));
        addMillisecondsToSystemCounter(millisElaspedAtInterruptCall);

        buttonPress(btn);
        // btn->currentState = PRESSED;
        if (btn->eventPress)
            btn->eventPress();
    }
    // Otherwise if button was last considered pressed and is now released.
    else if (btn->currentState == PRESSED && !isPressed(btn->buttonPin, BUTTON_IN) && buttonTimingCorrect(btn)) 
    {
         //Calculate milliseconds elasped since last clock call.
        uint16_t  millisElaspedAtInterruptCall = (TCNT1 * 500 / (TIMER1_TICKS_PER_SECOND_1024 / 2));
        addMillisecondsToSystemCounter(millisElaspedAtInterruptCall);

        buttonRelease(btn);
        // btn->currentState = RELEASED;
        if (btn->eventRelease)
            btn->eventRelease();
    }
}

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
    displaySetData = displayTime;

    while (1)
    {
        currentState = stateMachine.currentState;

        if (buttonInterruptTriggered) {
            buttonInterruptTriggered = false;

            updateAllButtons();
        }

        FSMUpdate(&stateMachine, fsmTransitionCallback);
    }

    return 0;
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
    //Add 500 milliseconds - any millis already accounted for previously to the system counter.
    addMillisecondsToSystemCounter(500 - millisAccountedBetweenTicks);
}

volatile uint32_t adcValue = 0;

ISR(TIMER0_COMPA_vect)
{
    displaySetData();

    displayUpdate(displayData);
}

// ADC/potentiometer has an issue where if it reaches 0x03 it cannot be raised again. 
// So I wont allow it to go below 0x4.
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
    if (BUTTON_IN)
        buttonInterruptTriggered = true;

}