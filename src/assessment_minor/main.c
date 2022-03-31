#include "avr/interrupt.h"
#include "avr/io.h"

#include "adc.h"
#include "button.h"
#include "clock.h"
#include "debug/debug_display.h"
#include "eeprom_data.h"
#include "fsm.h"
#include "led.h"
#include "null.h"
#include "sevenSegmentDisplay.h"
#include "systemtimer.h"
#include "timemode.h"
#include "timerTask.h"
#include "timers.h"

/// TIMING
#define TIMER1_TICKS_PER_SECOND_256PRESCALE     F_CPU / 256
#define TIMER1_TICKS_PER_100_MILLIS_256PRESCALE TIMER1_TICKS_PER_SECOND_256PRESCALE / 10
#define TIMER1_PERIOD_MILLISECONDS              100

#define TIMER2_TICKS_PER_SECOND_64_PRESCALE     F_CPU / 64
#define TIMER2_TICKS_PER_MILLISEC_64_PRESCALE   TIMER2_TICKS_PER_SECOND_64_PRESCALE / 250
#define TIMER2_PERIOD_MILLISECONDS              1

#define DISPLAY_CUTOFF                          0xff

FSM_TRANSITION_TABLE *stateMachinePtr = 0;

volatile Clock clock;
volatile bool alarmEnabled = false;
bool alarmSounding         = false;
volatile Clock alarm;

TimerTask *buzzerDisable          = NULL;
TimerTask *timerTaskBlinkAlarmLed = NULL;
TimerTask *timerTaskMainClock     = NULL;

TimeMode timeMode = TWENTY_FOUR_HOUR_TIME;

LED ledAlarm;
LED ledAlarmDisplayIndicator;
LED ledDebug;

Button *buttonSet       = NULL;
Button *buttonIncrement = NULL;
Button *buttonDecrement = NULL;

DisplayData displayData = {
    {0, 0, 0, 0}
};

volatile bool updateTimeDisplay = false;
volatile bool withDp            = true;

volatile bool buttonInterruptTriggered = false;
volatile bool shouldUpdateDisplay      = false;
void toggleDecimalPlaceDisplay() { withDp = !withDp; }

// Prototypes
void toggleDecimalPlaceDisplay();

FSMAction actionToggleAlarmLed();
FSMAction actionIncrementSecond();

FSMAction actionEnableAlarm();

bool displayPressed();
bool setPressed();
bool incrementPressed();
bool decrementPressed();
// Prototypes

void enableTimers() {
    // Primary millisecond counter
    enableTimer(TC2, TIMER2_CLOCK_SELECT_64_PRESCALER);
    // 7 Segment Display Update
    enableTimer(TC0, CLOCK_SELECT_1024_PRESCALER);
    // Buzzer
    enableTimer(TC1, CLOCK_SELECT_1_PRESCALER);
}

/// BUTTONS




/// BUTTONS

/// DISPLAY



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
    OCR1A = 1000;
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
    TIMSK2 = (1 << OCIE2A);
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

TimerTask *timerTaskIncrement;
TimerTask *timerTaskDecrement;
void incrementHoldTimerStart() {
    timerTaskIncrement->elaspedTime = timerTaskIncrement->period;
    TimerTaskEnable(timerTaskIncrement);
}
bool incrementTimerFired = false;
bool hasIncTimerFired() {
    if (incrementTimerFired) {
        incrementTimerFired = false;
        return true;
    }
    return false;
}
void incrementTime() {
    // If the button is not still held, stop the timer.
    if (buttonIncrement->currentState != HELD) {
        TimerTaskDisable(timerTaskIncrement);
        return;
    }
    incrementTimerFired = true;
}

void decrementHoldTimerStart() {
    timerTaskDecrement->elaspedTime = timerTaskDecrement->period;
    TimerTaskEnable(timerTaskDecrement);
}
bool decrementTimerFired = false;
void decrementTime() {
    // If the button is not still held, stop the timer.
    if (buttonDecrement->currentState != HELD) {
        TimerTaskDisable(timerTaskDecrement);
        return;
    }
    decrementTimerFired = true;
}
bool hasDecTimerFired() {
    if (decrementTimerFired) {
        decrementTimerFired = false;
        return true;
    }
    return false;
}

FSMAction actionClearButtonsTimersAndFlags() {}

void initialiseButtons() {

    buttonSet       = ButtonCreate(&DDRC, &PORTC, &PINC, PC1, NULL, NULL, true, NULL);
    buttonIncrement = ButtonCreate(&DDRC, &PORTC, &PINC, PC2, NULL, incrementHoldTimerStart, true, NULL);
    buttonDecrement = ButtonCreate(&DDRC, &PORTC, &PINC, PC3, NULL, decrementHoldTimerStart, true, NULL);
}

void initialiseTimerTasks() {
    // Create a timer task which increments the seconds, once every second.
    timerTaskMainClock = TimerTaskCreate(1000L, actionIncrementSecond, 0, true, false);

    // Create a timer task which toggle the decimal place display every 500ms.
    TimerTaskCreate(500L, toggleDecimalPlaceDisplay, 0, true, false);

    // Timer task for alarm set mode (led blink)
    timerTaskBlinkAlarmLed = TimerTaskCreate(500L, actionToggleAlarmLed, NULL, false, false);

    timerTaskIncrement = TimerTaskCreate(1000L, incrementTime, NULL, false, false);
    timerTaskDecrement = TimerTaskCreate(1000L, decrementTime, NULL, false, false);

    // Create a timer task for disabling the buzzer. Runs for 5 seconds,
    // disables buzzer and then is disabled (because it is created as a 1-shot
    // task).
    // buzzerDisable = TimerTaskCreate(5000L, disableBuzzer, 0, false, true);
}

void initialiseLeds() {
    ledAlarm                 = LED_Create(&DDRB, &PORTB, PB3);
    ledAlarmDisplayIndicator = LED_Create(&DDRB, &PORTB, PB2);

    ledDebug = LED_Create(&DDRB, &PORTB, PB5);
}

void loadDataFromEEPROM(FSM_STATE *currentState, volatile Clock *clk, volatile Clock *alm, volatile bool *almEn, TimeMode *tm) {
    EEPROMData data = EEPROM_ReadData();
    clk->hours      = data.timeHour;
    clk->minutes    = data.timeMinute;
    clk->seconds    = data.timeSecond;

    Clock_ValidateTime(clk);

    alm->hours   = data.alarmHour;
    alm->minutes = data.alarmMinute;

    Clock_ValidateTime(alm);

    *almEn = data.alarmEnable;
    if (*almEn) {
        actionEnableAlarm();
    }

    *tm           = data.timeMode;
    *currentState = data.currentState;
}

void saveDataToEEPROM(FSM_STATE *currentState, volatile Clock *clk, volatile Clock *alm, volatile bool *almEn, TimeMode *tm) {
    EEPROMData data = {0, clk->hours, clk->minutes, clk->seconds, alm->hours, alm->minutes, *almEn, *currentState, *tm};
    EEPROM_WriteData(&data);
}

void initialise() {
    SevenSegmentInitialise(&DISPLAY_DDR, DISPLAY_DATA_IN, &DISPLAY_PORT, &DISPLAY_CLOCK_DDR, &DISPLAY_CLOCK_PORT, DISPLAY_SHIFT_PIN, DISPLAY_LATCH_PIN);
    initialiseButtons();
    initialiseTimer2();
    initialiseTimer1();
    initialiseTimer0();
    initialiseADC();
    initialiseLeds();
    initialiseTimerTasks();
}

/// INITIALISERS

// FSM Triggers

FSMTrigger setPressed() { return ButtonReadFlag(buttonSet) == FLAG_PRESSED; }

FSMTrigger setHeld() { return ButtonReadFlag(buttonSet) == FLAG_HELD; }

FSMTrigger incrementPressed() { return ButtonReadFlag(buttonIncrement) == FLAG_PRESSED; }

FSMTrigger incrementHeld() { return ButtonReadFlag(buttonIncrement) == FLAG_HELD; }

FSMTrigger decrementPressed() { return ButtonReadFlag(buttonDecrement) == FLAG_PRESSED; }

FSMTrigger decrementHeld() { return ButtonReadFlag(buttonDecrement) == FLAG_HELD; }

// Aliases
FSMTrigger displayPressed() { return decrementPressed(); }
FSMTrigger displayHeld() { return decrementHeld(); }
FSMTrigger alarmPressed() { return incrementPressed(); }
FSMTrigger alarmHeld() { return incrementHeld(); }

// If increment has been pressed or increment time task has elasped.
FSMTrigger incrementPressedOrHeld() { return incrementPressed() || hasIncTimerFired(); }

FSMTrigger decrementPressedOrHeld() { return decrementPressed() || hasDecTimerFired(); }

FSMTrigger alarmSetHeld() {
    bool res = setHeld() && buttonIncrement->currentState == PRESSED;
    return res;
}

FSMTrigger alarmTrigger() { return (Clock_CompareClocks(&clock, &alarm) == EQUAL) && !alarmSounding; }

// FSM Trigger

// FSM Transition Actions

FSMAction actionIncrementHour() { Clock_AddHours(&clock, 1); }

FSMAction actionDecrementHour() { Clock_AddHours(&clock, -1); }

FSMAction actionIncrementMinute() { Clock_AddMinutes(&clock, 1); }

FSMAction actionDecrementMinute() { Clock_AddMinutes(&clock, -1); }

bool updateEeprom = false;
FSMAction actionIncrementSecond() {
    Clock_AddSeconds(&clock, 1);
    updateEeprom = true;
}

FSMAction actionIncrementHourAlarm() { Clock_AddHours(&alarm, 1); }

FSMAction actionDecrementHourAlarm() { Clock_AddHours(&alarm, -1); }

FSMAction actionIncrementMinuteAlarm() { Clock_AddMinutes(&alarm, 1); }

FSMAction actionDecrementMinuteAlarm() { Clock_AddMinutes(&alarm, -1); }

FSMAction actionToggleTimeDisplayModes() { timeMode = (timeMode == TWELVE_HOUR_TIME ? TWENTY_FOUR_HOUR_TIME : TWELVE_HOUR_TIME); }

FSMAction actionEnableAlarm() {
    TimerTaskDisable(timerTaskBlinkAlarmLed);
    alarmEnabled = true;
    LED_On(&ledAlarm);
}

FSMAction actionToggleAlarm() {
    alarmEnabled = !alarmEnabled;
    LED_Toggle(&ledAlarm);
}

FSMAction actionToggleAlarmLed() { LED_Toggle(&ledAlarm); }

FSMAction actionToggleAlarmIndicator() { LED_Toggle(&ledAlarmDisplayIndicator); }

FSMAction actionAlarmStart() { alarmSounding = true; }

FSMAction actionAlarmStop() { alarmSounding = false; }

FSM_STATE stateBeforeAlarmMode = DISPLAY_HH_MM;
FSM_STATE getFSMStateForAlarmExit() { return stateBeforeAlarmMode; }
FSMAction actionAlarmSetModeEnter() {
    // Save original state.
    stateBeforeAlarmMode = stateMachinePtr->currentState;

    // Clear input timers and button flags
    TimerTaskDisable(timerTaskIncrement);
    TimerTaskReset(timerTaskIncrement);
    ButtonClearFlagsAndForceToReleased();

    // Enable blink for alarm set indicator
    timerTaskBlinkAlarmLed->elaspedTime = timerTaskBlinkAlarmLed->period;
    TimerTaskEnable(timerTaskBlinkAlarmLed);
}

FSMAction pauseMainClock() { TimerTaskDisable(timerTaskMainClock); }
FSMAction resumeMainClock() { TimerTaskEnable(timerTaskMainClock); }

// FSM Transition Actions

/// Display Functions

void displayFunctionBlank() {
    displayData.data[SEG_FAR_RIGHT] = mapChar(BLANK); // Far Right
    displayData.data[SEG_RIGHT]     = mapChar(BLANK); // Centre Right
    displayData.data[SEG_LEFT]      = mapChar(BLANK); // Center Left
    displayData.data[SEG_FAR_LEFT]  = mapChar(BLANK); // Far Left
}

uint8_t determineHoursValueFor(uint8_t hours) {
    // Are we 12 hour time and after 12 midday? If so subtract 12.
    if (timeMode == TWELVE_HOUR_TIME && hours > 12) {
        hours -= 12;
    }

    // If we are after midnight, after subtraction hours will be at 0, move to 12 for display.
    if (timeMode == TWELVE_HOUR_TIME && hours == 0) {
        hours = 12;
    }
    return hours;
}

void displayClockHHMMOnSegments(volatile Clock *clk) {
    // If the pointer is null, show blank screen.
    // Maybe light up all segments???
    if (!clk) {
        displayFunctionBlank();
        return;
    }

    uint8_t ml = clk->minutes % 10;
    uint8_t mh = clk->minutes / 10;

    // 12 hour time adjustment.
    uint8_t hours = determineHoursValueFor(clk->hours);

    uint8_t hl = hours % 10;
    uint8_t hh = hours / 10;

    displayData.data[SEG_FAR_RIGHT] = mapChar(ml); // Far Right
    displayData.data[SEG_RIGHT]     = mapChar(mh); // Centre Right
    displayData.data[SEG_LEFT]      = mapChar(hl); // Center Left
    displayData.data[SEG_FAR_LEFT]  = mapChar(hh); // Far Left

    // Decimal Point
    displayData.data[SEG_LEFT] &= (withDp ? mapChar(DP) : mapChar(BLANK));

    // PM Indicator: Active if showing 12 hours time and hours are > 12.
    displayData.data[SEG_FAR_RIGHT] &= (timeMode == TWELVE_HOUR_TIME && clk->hours > 12 ? mapChar(DP) : mapChar(BLANK));
}

void displayFunctionTimeHHMM() { displayClockHHMMOnSegments(&clock); }

void displayFunctionAlarmHHMM() { displayClockHHMMOnSegments(&alarm); }

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
    displayData.data[SEG_FAR_RIGHT] &= (timeMode == TWELVE_HOUR_TIME && clock.hours > 12 ? mapChar(DP) : mapChar(BLANK));
}

void displaySetHoursForClock(volatile Clock *clk) {
    if (!clk) {
        displayFunctionBlank();
        return;
    }

    uint8_t hours = determineHoursValueFor(clk->hours);

    uint8_t hl = hours % 10;
    uint8_t hh = hours / 10;

    displayData.data[SEG_FAR_RIGHT] = mapChar(BLANK); // Far Right
    displayData.data[SEG_RIGHT]     = mapChar(BLANK); // Centre Right
    displayData.data[SEG_LEFT]      = mapChar(hl);    // Center Left
    displayData.data[SEG_FAR_LEFT]  = mapChar(hh);    // Far Left

    // PM Indicator
    displayData.data[3] &= (timeMode == TWELVE_HOUR_TIME && clk->hours > 12 ? mapChar(DP) : mapChar(BLANK));
}

void displaySetMinutesForClock(volatile Clock *clk) {
    if (!clk) {
        displayFunctionBlank();
        return;
    }

    uint8_t ml = clk->minutes % 10;
    uint8_t mh = clk->minutes / 10;

    displayData.data[SEG_FAR_RIGHT] = mapChar(ml);    // Far Right
    displayData.data[SEG_RIGHT]     = mapChar(mh);    // Centre Right
    displayData.data[SEG_LEFT]      = mapChar(BLANK); // Center Left
    displayData.data[SEG_FAR_LEFT]  = mapChar(BLANK); // Far Left
}

void displayFunctionSetTimeHH() { displaySetHoursForClock(&clock); }

void displayFunctionSetTimeMM() { displaySetMinutesForClock(&clock); }

void displayFunctionSetAlarmHH() { displaySetHoursForClock(&alarm); }

void displayFunctionSetAlarmMM() { displaySetMinutesForClock(&alarm); }

/// Display Functions

void resetSeconds() { clock.seconds = 0; }

/// Finite State Machine Transitions

FSM_TRANSITION displayHoursToDisplayMinutes = {DISPLAY_HH_MM, displayPressed, noAction, &STATE_MM_SS};                   // HH:MM -> MM:SS when Display Pressed.
FSM_TRANSITION displayHoursToSetTime        = {DISPLAY_HH_MM, setPressed, pauseMainClock, &STATE_MODE_HR};               // HH:MM -> Set HH when Set Pressed.
FSM_TRANSITION displayHoursToggleTimeMode   = {DISPLAY_HH_MM, setHeld, actionToggleTimeDisplayModes, &STATE_HH_MM};      // HH:MM -> HH:MM, change 12/24hr mode when Display Held.
FSM_TRANSITION displayHoursToggleAlarm      = {DISPLAY_HH_MM, incrementPressed, actionToggleAlarm, &STATE_HH_MM};        // HH:MM -> HH:MM, enable alarm when Increment Pressed.
FSM_TRANSITION displayHoursToSetAlarm       = {DISPLAY_HH_MM, alarmSetHeld, actionAlarmSetModeEnter, &STATE_ALARM_HR}; // HH:MM -> Set Alarm HH when Alarm Set Held (Set held + Alarm/increment pressed)

FSM_TRANSITION alarmSetHHToAlarmSetMM = {ALARM_SET_HR, setPressed, noAction, &STATE_ALARM_MIN};                            // Alarm Set HH -> Alarm Set MM, when Set Pressed.
FSM_TRANSITION alarmSetHHIncrement    = {ALARM_SET_HR, incrementPressedOrHeld, actionIncrementHourAlarm, &STATE_ALARM_HR}; // Alarm Set HH -> Alarm Set HH, increment hour, when Increment Pressed.
FSM_TRANSITION alarmSetHHDecrement    = {ALARM_SET_HR, decrementPressedOrHeld, actionDecrementHourAlarm, &STATE_ALARM_HR}; // Alarm Set HH -> Alarm Set HH, decrement hour, when Decrement Pressed.

// MUST RETURN TO LAST DISPLAY MODE, NEED TO STORE TEMP VARIABLE ETC ETC
FSM_TRANSITION alarmSetMMToLastDisplayMode = {ALARM_SET_MIN, setPressed, actionEnableAlarm, &getFSMStateForAlarmExit};              // Alarm Set Min -> HH:MM, enables alarm, when Set Pressed.
FSM_TRANSITION alarmSetMMIncrement         = {ALARM_SET_MIN, incrementPressedOrHeld, actionIncrementMinuteAlarm, &STATE_ALARM_MIN};  // Alarm Set Min -> Alarm Set Hin, increment minute, when Increment Pressed.
FSM_TRANSITION alarmSetMMDecrement         = {ALARM_SET_MIN, decrementPressedOrHeld, actionDecrementMinuteAlarm, &STATE_ALARM_MIN}; // Alarm Set Min -> Alarm Set Hin, decrement minute, when Decrement Pressed.

FSM_TRANSITION displayMinutesToSetAlarm     = {DISPLAY_MM_SS, alarmSetHeld, actionAlarmSetModeEnter, &STATE_ALARM_HR};      // HH:MM -> Set Alarm HH when Alarm Set Held (Set held + Alarm/increment pressed)
FSM_TRANSITION displayMinutesToDisplayAlarm = {DISPLAY_MM_SS, displayPressed, actionToggleAlarmIndicator, &STATE_DISP_ALARM}; // MM:SS -> Alarm HH:MM, toggle alarm indicator, when Display Pressed.
FSM_TRANSITION displayMinutesToggleAlarm    = {DISPLAY_MM_SS, incrementPressed, actionToggleAlarm, &STATE_MM_SS};             // MM:SS -> MM:SS, toggle alarm (enable/disable), when Increment Pressed.

FSM_TRANSITION displayAlarmToSetAlarm    = {DISPLAY_ALARM, alarmSetHeld, actionAlarmSetModeEnter, &STATE_ALARM_HR}; // HH:MM -> Set Alarm HH when Alarm Set Held (Set held + Alarm/increment pressed)
FSM_TRANSITION displayAlarmToDisplayHHMM = {DISPLAY_ALARM, displayPressed, actionToggleAlarmIndicator, &STATE_HH_MM}; // Alarm HH:MM -> HH:MM, toggle alarm indicator, when Display Presed.
FSM_TRANSITION displayAlarmToggleAlarm   = {DISPLAY_ALARM, incrementPressed, actionToggleAlarm, &STATE_DISP_ALARM};   // Alarm HH:MM -> Alarm HH:MM, toggle alarm (enable/disable), when Increment Pressed.

FSM_TRANSITION timeSetHrToMin     = {SET_TIME_MODE_HR, setPressed, noAction, &STATE_MODE_MIN};                       // Set HH -> Set MM, when Set Pressed.
FSM_TRANSITION timeSetHrIncrement = {SET_TIME_MODE_HR, incrementPressedOrHeld, actionIncrementHour, &STATE_MODE_HR}; // Set HH -> Set HH, increment hour, when Increment Pressed or Held.
FSM_TRANSITION timeSetHrDecrement = {SET_TIME_MODE_HR, decrementPressedOrHeld, actionDecrementHour, &STATE_MODE_HR}; // Set HH -> Set HH, decrement  hour, when Decrement Pressed or Held.

FSM_TRANSITION timeSetMinToDisplayHr = {SET_TIME_MODE_MIN, setPressed, resumeMainClock, &STATE_HH_MM};                      // Set MM -> HH:MM, when Set Pressed.
FSM_TRANSITION timeSetMinIncrement   = {SET_TIME_MODE_MIN, incrementPressedOrHeld, actionIncrementMinute, &STATE_MODE_MIN}; // Set MM -> Set MM, increment minute, when Increment Pressed or Held.
FSM_TRANSITION timeSetMinDecrement   = {SET_TIME_MODE_MIN, decrementPressedOrHeld, actionDecrementMinute, &STATE_MODE_MIN}; // Set MM -> Set MM, decrement  minute, when Decrement Pressed or Held.

/// Finite State Machine Transitions

DisplayFunctionPointer displayFunctions[FSM_STATE_COUNT] = {displayFunctionTimeHHMM, displayFunctionTimeMMSS, displayFunctionSetTimeHH, displayFunctionSetTimeMM, displayFunctionAlarmHHMM, displayFunctionSetAlarmHH, displayFunctionSetAlarmMM};

// #define OVERRIDE_DISPLAY_FUNCTION

void disableBuzzer() { disableTimer(TC2); }

int main() {
    // Initialise all system components.
    initialise();

    // Read in configuration for time, clock and state.
    // Clock_AddTime(&clock, 00, 43, 44);
    // Clock_AddTime(&alarm, 7, 30, 0);

    // Setup the state machine transition table.
    FSM_TRANSITION_TABLE stateMachine = {
        DISPLAY_HH_MM,
        {displayHoursToDisplayMinutes, displayHoursToSetAlarm, displayMinutesToDisplayAlarm, displayAlarmToDisplayHHMM, displayHoursToggleAlarm, displayMinutesToggleAlarm, displayAlarmToggleAlarm, displayHoursToSetTime, timeSetHrToMin,
          timeSetMinToDisplayHr, displayHoursToggleTimeMode, timeSetHrIncrement, timeSetHrDecrement, timeSetMinIncrement, timeSetMinDecrement, alarmSetHHToAlarmSetMM, alarmSetHHIncrement, alarmSetHHDecrement,
          alarmSetMMToLastDisplayMode, alarmSetMMIncrement, alarmSetMMDecrement, displayMinutesToSetAlarm, displayAlarmToSetAlarm}
    };
    stateMachinePtr = &stateMachine;

    // loadDataFromEEPROM(&stateMachine.currentState, &clock, &alarm, &alarmEnabled, &timeMode);

    enableTimers();

    // Enable interrupts.
    sei();

#ifdef OVERRIDE_DISPLAY_FUNCTION
    displayFunction = displayFunctionCurrentState;
#endif

    while (1) {
        // Update the FSM, and upon a state change clear all button input flags.
        if (FSMUpdate(&stateMachine) == STATE_CHANGE) {
            ButtonClearAllFlags();
        }

#ifndef OVERRIDE_DISPLAY_FUNCTION
        displayFunction = displayFunctions[stateMachine.currentState];
#endif

        if (shouldUpdateDisplay) {
            shouldUpdateDisplay = false;
            // If the ADC reports a maxmimum value, turn the display off. Otherwise call the displayFunction, if set.
            if (adcValue >= DISPLAY_CUTOFF) {
                displayFunctionBlank();
            } else {
                if (displayFunction)
                    displayFunction();
            }
            // Push the data to the display.
            SevenSegmentUpdate(displayData.data);
        }

        if (updateEeprom) {
            updateEeprom = false;
            // saveDataToEEPROM(&stateMachinePtr->currentState, &clock, &alarm, &alarmEnabled, &timeMode);
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

    TimerTaskUpdate(TIMER2_PERIOD_MILLISECONDS);

    isrTimeEnd = TCNT2;
}

ISR(ADC_vect) {
    // Set timer output compare to ADC value
    adcValue = OCR0A = ADCH;
}

// Button interrupt register
ISR(PCINT1_vect) { ButtonUpdateAll(); }