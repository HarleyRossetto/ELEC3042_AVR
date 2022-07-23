#include "avr/interrupt.h"
#include "avr/io.h"

#include "adc.h"
#include "button.h"
#include "clock.h"
#include "flag.h"
#include "fsm.h"
#include "led.h"
#include "null.h"
#include "sevenSegmentDisplay.h"
#include "systemtimer.h"
#include "timemode.h"
#include "timerTask.h"
#include "timers.h"

#include "eeprom.h"


// Used to determine at what ADC reading we turn the display off.
#define DISPLAY_CUTOFF 0xff

// typedef to annotate initialiser functions.
typedef void Initialiser;

typedef void (*DisplayFunctionPointer)();

/*
    Keeps track of how many times the milliseconds timer/counter (TCC2) is triggered.
    In the event the system misses the chance to updated timed/scheduled tasks we can use this value
    to add 1 (ms) * timer2InterruptCount to determine the delta since tasks where last updated.
    This value will be reset to zero after updating tasks.
 */
uint8_t timer2InterruptCount = 0;

// We need to keep a pointer to the state machine struct for use an FSMAction callback.
// See: FSMAction actionAlarmSetModeEnter();
FSM_TRANSITION_TABLE *stateMachinePtr = NULL;

// The system clock that keeps track of time.
volatile Clock clock;
// The alarm clock which maintains the alarms set value.
volatile Clock alarm;

// Time mode enumeration, 12/24hrs
TimeMode timeMode = TWELVE_HOUR_TIME;

TimerTask *timerTaskMainClock         = NULL;
TimerTask *timerTaskIncrement         = NULL;
TimerTask *timerTaskDecrement         = NULL;
TimerTask *timerTaskBuzzerDisable     = NULL;
TimerTask *timerTaskBuzzerPeriod      = NULL;
TimerTask *timerTaskBlinkAlarmLed     = NULL;
TimerTask *timerTaskAlarmReenableHold = NULL;
TimerTask *timerTaskUpdateEeprom      = NULL;

LED ledAlarm;
LED ledAlarmDisplayIndicator;

Button *buttonSet       = NULL;
Button *buttonIncrement = NULL;
Button *buttonDecrement = NULL;

DisplayData displayData = {
    {0, 0, 0, 0}
};

Flag flag_updateDisplay;
Flag flag_updateTimers;
Flag flag_alarmSounding;
Flag flag_alarmEnabled;
Flag flag_alarmReenable;

volatile bool withDp            = true;

DisplayFunctionPointer displayFunction;

void toggleDecimalPlaceDisplay() { withDp = !withDp; }

FSM_STATE stateBeforeAlarmMode = DISPLAY_HH_MM;

bool incrementTimerFired = false;
bool decrementTimerFired = false;

////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                //
//                                      Prototypes                                                //
//                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

Initialiser initialiseFlags();
Initialiser initialiseTimer2();
Initialiser initialiseTimer1();
Initialiser initialiseTimer0();
Initialiser initialiseADC();
Initialiser initialiseButtons();
Initialiser initialiseTimerTasks();
Initialiser initialiseLeds();
Initialiser initialiseSystemState();
Initialiser initialiseStateMachine();
Initialiser initialise();

void updateDisplay();
void updateEeprom();
void updateTimers();

FSMAction actionIncrementSecond();
FSMAction actionEnableAlarm();

bool displayPressed();
bool setPressed();
bool incrementPressed();
bool decrementPressed();

void enableTimers();

void incrementHoldTimerStart();
void decrementHoldTimerStart();
bool hasIncTimerFired();
bool hasDecTimerFired();

void incrementTime();
void decrementTime();

void enableAlarmTimer();
void disableAlarmTimer();
void alternateAlarm();
void alarmActivate();
void alarmDeactivate();

FSM_STATE getFSMStateForAlarmExit() { return stateBeforeAlarmMode; }

FSMTrigger setPressed();
FSMTrigger setHeld();
FSMTrigger incrementPressed();
FSMTrigger incrementHeld();
FSMTrigger decrementPressed();
FSMTrigger decrementHeld();
FSMTrigger displayPressed();
FSMTrigger displayHeld();
FSMTrigger alarmPressed();
FSMTrigger alarmHeld();
FSMTrigger incrementPressedOrHeld();
FSMTrigger decrementPressedOrHeld();
FSMTrigger alarmSetHeld();
FSMTrigger shouldAlarmTrigger();
FSMTrigger clearSoundingAlarm();

FSMAction actionIncrementHour();
FSMAction actionDecrementHour();
FSMAction actionIncrementMinute();
FSMAction actionDecrementMinute();
FSMAction actionIncrementSecond();
FSMAction actionIncrementHourAlarm();
FSMAction actionDecrementHourAlarm();
FSMAction actionIncrementMinuteAlarm();
FSMAction actionDecrementMinuteAlarm();
FSMAction actionToggleTimeDisplayModes();
FSMAction actionEnableAlarm();
FSMAction actionToggleAlarm();
FSMAction actionToggleAlarmIndicator();
FSMAction actionAlarmSetModeEnter();
FSMAction pauseMainClock();
FSMAction resumeMainClock();


////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                //
//                             Finite State Machine Transitions                                   //
//                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

FSM_TRANSITION displayHoursSilenceAlarm     = {DISPLAY_HH_MM, clearSoundingAlarm, alarmDeactivate, &STATE_HH_MM};      // HH:MM -> MM:SS when Display Pressed.
FSM_TRANSITION displayHoursToDisplayMinutes = {DISPLAY_HH_MM, displayPressed, noAction, &STATE_MM_SS};                 // HH:MM -> MM:SS when Display Pressed.
FSM_TRANSITION displayHoursToSetTime        = {DISPLAY_HH_MM, setPressed, pauseMainClock, &STATE_MODE_HR};             // HH:MM -> Set HH when Set Pressed.
FSM_TRANSITION displayHoursToggleTimeMode   = {DISPLAY_HH_MM, setHeld, actionToggleTimeDisplayModes, &STATE_HH_MM};    // HH:MM -> HH:MM, change 12/24hr mode when Display Held.
FSM_TRANSITION displayHoursToggleAlarm      = {DISPLAY_HH_MM, incrementPressed, actionToggleAlarm, &STATE_HH_MM};      // HH:MM -> HH:MM, enable alarm when Increment Pressed.
FSM_TRANSITION displayHoursToSetAlarm       = {DISPLAY_HH_MM, alarmSetHeld, actionAlarmSetModeEnter, &STATE_ALARM_HR}; // HH:MM -> Set Alarm HH when Alarm Set Held (Set held + Alarm/increment pressed)

FSM_TRANSITION alarmSetHHSilenceAlarm = {ALARM_SET_HR, clearSoundingAlarm, alarmDeactivate, &STATE_ALARM_HR};
FSM_TRANSITION alarmSetHHToAlarmSetMM = {ALARM_SET_HR, setPressed, noAction, &STATE_ALARM_MIN};                            // Alarm Set HH -> Alarm Set MM, when Set Pressed.
FSM_TRANSITION alarmSetHHIncrement    = {ALARM_SET_HR, incrementPressedOrHeld, actionIncrementHourAlarm, &STATE_ALARM_HR}; // Alarm Set HH -> Alarm Set HH, increment hour, when Increment Pressed.
FSM_TRANSITION alarmSetHHDecrement    = {ALARM_SET_HR, decrementPressedOrHeld, actionDecrementHourAlarm, &STATE_ALARM_HR}; // Alarm Set HH -> Alarm Set HH, decrement hour, when Decrement Pressed.

FSM_TRANSITION alarmSetMMSilenceAlarm      = {ALARM_SET_MIN, clearSoundingAlarm, alarmDeactivate, &STATE_ALARM_MIN};
FSM_TRANSITION alarmSetMMToLastDisplayMode = {ALARM_SET_MIN, setPressed, actionEnableAlarm, &getFSMStateForAlarmExit};              // Alarm Set Min -> HH:MM, enables alarm, when Set Pressed.
FSM_TRANSITION alarmSetMMIncrement         = {ALARM_SET_MIN, incrementPressedOrHeld, actionIncrementMinuteAlarm, &STATE_ALARM_MIN}; // Alarm Set Min -> Alarm Set Hin, increment minute, when Increment Pressed.
FSM_TRANSITION alarmSetMMDecrement         = {ALARM_SET_MIN, decrementPressedOrHeld, actionDecrementMinuteAlarm, &STATE_ALARM_MIN}; // Alarm Set Min -> Alarm Set Hin, decrement minute, when Decrement Pressed.

FSM_TRANSITION displayMinutesSilenceAlarm   = {DISPLAY_MM_SS, clearSoundingAlarm, alarmDeactivate, &STATE_MM_SS};
FSM_TRANSITION displayMinutesToSetAlarm     = {DISPLAY_MM_SS, alarmSetHeld, actionAlarmSetModeEnter, &STATE_ALARM_HR};        // HH:MM -> Set Alarm HH when Alarm Set Held (Set held + Alarm/increment pressed)
FSM_TRANSITION displayMinutesToDisplayAlarm = {DISPLAY_MM_SS, displayPressed, actionToggleAlarmIndicator, &STATE_DISP_ALARM}; // MM:SS -> Alarm HH:MM, toggle alarm indicator, when Display Pressed.
FSM_TRANSITION displayMinutesToggleAlarm    = {DISPLAY_MM_SS, incrementPressed, actionToggleAlarm, &STATE_MM_SS};             // MM:SS -> MM:SS, toggle alarm (enable/disable), when Increment Pressed.

FSM_TRANSITION displayAlarmSilenceAlarm  = {DISPLAY_ALARM, clearSoundingAlarm, alarmDeactivate, &STATE_DISP_ALARM};
FSM_TRANSITION displayAlarmToSetAlarm    = {DISPLAY_ALARM, alarmSetHeld, actionAlarmSetModeEnter, &STATE_ALARM_HR};   // HH:MM -> Set Alarm HH when Alarm Set Held (Set held + Alarm/increment pressed)
FSM_TRANSITION displayAlarmToDisplayHHMM = {DISPLAY_ALARM, displayPressed, actionToggleAlarmIndicator, &STATE_HH_MM}; // Alarm HH:MM -> HH:MM, toggle alarm indicator, when Display Presed.
FSM_TRANSITION displayAlarmToggleAlarm   = {DISPLAY_ALARM, incrementPressed, actionToggleAlarm, &STATE_DISP_ALARM};   // Alarm HH:MM -> Alarm HH:MM, toggle alarm (enable/disable), when Increment Pressed.

FSM_TRANSITION timeSetHrSilenceAlarm = {SET_TIME_MODE_HR, clearSoundingAlarm, alarmDeactivate, &STATE_MODE_HR};
FSM_TRANSITION timeSetHrToMin        = {SET_TIME_MODE_HR, setPressed, noAction, &STATE_MODE_MIN};                       // Set HH -> Set MM, when Set Pressed.
FSM_TRANSITION timeSetHrIncrement    = {SET_TIME_MODE_HR, incrementPressedOrHeld, actionIncrementHour, &STATE_MODE_HR}; // Set HH -> Set HH, increment hour, when Increment Pressed or Held.
FSM_TRANSITION timeSetHrDecrement    = {SET_TIME_MODE_HR, decrementPressedOrHeld, actionDecrementHour, &STATE_MODE_HR}; // Set HH -> Set HH, decrement  hour, when Decrement Pressed or Held.

FSM_TRANSITION timeSetMinSilenceAlarm = {SET_TIME_MODE_MIN, clearSoundingAlarm, alarmDeactivate, &STATE_MODE_MIN};
FSM_TRANSITION timeSetMinToDisplayHr  = {SET_TIME_MODE_MIN, setPressed, resumeMainClock, &STATE_HH_MM};                      // Set MM -> HH:MM, when Set Pressed.
FSM_TRANSITION timeSetMinIncrement    = {SET_TIME_MODE_MIN, incrementPressedOrHeld, actionIncrementMinute, &STATE_MODE_MIN}; // Set MM -> Set MM, increment minute, when Increment Pressed or Held.
FSM_TRANSITION timeSetMinDecrement    = {SET_TIME_MODE_MIN, decrementPressedOrHeld, actionDecrementMinute, &STATE_MODE_MIN}; // Set MM -> Set MM, decrement  minute, when Decrement Pressed or Held.



////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                //
//                                          INITIALIZERS                                          //
//                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

Initialiser initialise() {
    InitialiseSevenSegmentDisplay(&DDRB, PB0, &PORTB, &DDRD, &PORTD, PD7, PD4);
    initialiseButtons();
    initialiseTimer2();
    initialiseTimer1();
    initialiseTimer0();
    initialiseADC();
    initialiseLeds();
    initialiseTimerTasks();
    initialiseFlags();
    EEPROM_Initialise();
}

Initialiser initialiseButtons() {
    buttonSet       = ButtonCreate(&DDRC, &PORTC, &PINC, PC1, NULL, NULL, true);
    buttonIncrement = ButtonCreate(&DDRC, &PORTC, &PINC, PC2, NULL, incrementHoldTimerStart, true);
    buttonDecrement = ButtonCreate(&DDRC, &PORTC, &PINC, PC3, NULL, decrementHoldTimerStart, true);
}

// Millisecond timer
Initialiser initialiseTimer2() {
    disableTimer(TC2);

    TCCR2A = (1 << WGM21) | (0 << WGM20);
    TCCR2B = (0 << WGM22);
    TCNT2  = 0;
    OCR2A  = 249; // Zero relative (0 - 249 = 250)
    OCR2B  = 0;
    TIMSK2 = (1 << OCIE2A);
    TIFR2  = 0x00;
}

/**
 * @brief   Prepares Timer 1 for use as the primary time-keeping clock.
 *          NOTE: This does not start the timer, enableTimer(TC1, PRESCALER);
 * must be called to do so.
 *
 * Timer 1 will run with a 100ms period using a 256 prescaler. With this
 * configuration no compensation is required because there are no fractions
 * involved in compare counts.
 */
Initialiser initialiseTimer1() {
    disableTimer(TC1);
    // Timer/Counter Control Register A (Compare Output Modes and Waveform
    // Generation Modes (bits 11 and 10))
    TCCR1A = TC1_TCCR1A_CFG; // Waveform Generation Mode - PWM, Phase Correct Mode 11

    // Timer starts once Clock Select is set, so we will configure that last.
    //  Timer/Counter Control Register B (Input Capture Noise Canceler, Input
    //  Capture Edge Select, Waveform Generation Mode (bits 13 and 12), and Clock
    //  Select)
    TCCR1B = (1 << WGM13) | (0 << WGM12);

    // Timer/Counter Control Register C (Force Output Compare)
    TCCR1C = 0x00;

    // Timer/Counter Value
    TCNT1 = 0;

    // Output Compare Register A
    OCR1A = 124;
    // Output Compare Register B
    OCR1B = 0;

    // Input Capture Register
    ICR1 = 0x0;

    // Timer/Counter Interrupt Mask Register (Input Capture Interrupt Enable,
    // Output Compare A/B Match Interrupt Enable, and Overflow Interrupt Enable)
    TIMSK1 = 0;

    // Timer/Counter Interrupt Flag Register (Input Capture, Output Compare A/B
    // and Overflow Flags)
    TIFR1 = 0x00; // Ensure all timer interrupt flags are cleared.

    // Enable output on pin PB1/pin 9
    DDRB |= (1 << PB1);
}

// Display Update Timer
Initialiser initialiseTimer0() {
    disableTimer(TC0);

    TCCR0A = (1 << WGM01) | (0 << WGM00);
    TCCR0B = (0 << WGM02);
    TCNT0  = 0;
    OCR0A  = 255;
    OCR0B  = 0;
    TIMSK0 = (1 << OCIE0A);
    TIFR0  = 0x00;
}

Initialiser initialiseADC() {
    // ADC Multiplexer Selection Register
    ADMUX = ADC_VREF_AVCC | ADC_CH_0 | (1 << ADLAR);

    // ADC Control and Status Register A
    // Enabled ADC, Enable Interrupts, Enable Auto Trigger, 128 prescaler
    ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADATE) | ADC_PRESCALER_32 | (1 << ADSC);

    // ADC Control and Status Register B
    // Set to free running
    // ADCSRB = ADC_AUTO_TRIGGER_FREE_RUNNING;
    ADCSRB = ADC_AUTO_TRIGGER_SOURCE_TCC0_COMPARE_MATCH_A;

    // ADC Data Register
    ADC = 0;

    // Digital Inout Disable Register
    DIDR0 = (1 << ADC0D); // Disable digital input on AD0
}

Initialiser initialiseLeds() {
    ledAlarm                 = LED_Create(&DDRB, &PORTB, PB3);
    ledAlarmDisplayIndicator = LED_Create(&DDRB, &PORTB, PB2);
}

Initialiser initialiseTimerTasks() {
    // Create a timer task which increments the seconds, once every second.
    timerTaskMainClock = TimerTaskCreate(1000L, actionIncrementSecond, 0, true, false);

    // Create a timer task which toggle the decimal place display every 500ms.
    TimerTaskCreate(500L, toggleDecimalPlaceDisplay, 0, true, false);

    // Timer task for alarm set mode (led blink)
    timerTaskBlinkAlarmLed = TimerTaskCreate(500L, LED_Toggle, &ledAlarm, false, false);

    // Increment timer task.
    timerTaskIncrement = TimerTaskCreate(1000L, incrementTime, NULL, false, false);
    // Dencrement timer task.
    timerTaskDecrement = TimerTaskCreate(1000L, decrementTime, NULL, false, false);

    // Create a timer task for disabling the buzzer. Runs for 5 seconds,
    // disables buzzer and then is disabled (because it is created as a 1-shot
    // task).
    timerTaskBuzzerDisable = TimerTaskCreate(5000L, alarmDeactivate, 0, false, true);
    // Create a timer task for alternating between buzzer on and off, every 1 second.
    timerTaskBuzzerPeriod = TimerTaskCreate(1000L, alternateAlarm, 0, false, false);

    // Create an EEPROM update task that runs every minute.
    // It will set the eeprom update flag upon execution.
    timerTaskUpdateEeprom = TimerTaskCreate(1000L * 10, &updateEeprom, NULL, true, false);

    // Alarm Reenable hold timer task prevents the alarm from restarting if it is cancelled within
    // the first second, where it's hours and minutes still match that of the main clock.
    timerTaskAlarmReenableHold = TimerTaskCreate(1001L, Flag_Clear, &flag_alarmReenable, false, true);
}

/**
 * @brief Initialise flags and their associated callbacks (if applicable).
 */
Initialiser initialiseFlags() {
    flag_updateDisplay = Flag_Create(&updateDisplay, NULL);
    flag_updateTimers  = Flag_Create(&updateTimers, NULL);
    flag_alarmSounding = Flag_Create(NULL, NULL);
    flag_alarmEnabled  = Flag_Create(NULL, NULL);
}

Initialiser initialiseSystemState() {
    // Load data from eeprom.
    EEPROMReadClockData cd = EEPROM_ReadClockData();
    // If the data is valid, configure the system state accordingly.
    if (cd.isValid) {
        clock.hours                   = cd.clockData.timeHour;
        clock.minutes                 = cd.clockData.timeMinute;
        clock.seconds                 = 0;
        alarm.hours                   = cd.clockData.alarmHour;
        alarm.minutes                 = cd.clockData.alarmMinute;
        alarm.seconds                 = 0;
        stateMachinePtr->currentState = cd.clockData.systemState;
        if (cd.clockData.alarmEnabled)
            actionToggleAlarm();
        if (cd.clockData.timeMode24Hour)
            actionToggleTimeDisplayModes();

        if (stateMachinePtr->currentState == ALARM_SET_HR || stateMachinePtr->currentState == ALARM_SET_MIN) {
            actionAlarmSetModeEnter();
            // Overwrite stateBeforeAlarmMode with that stored in eeprom because the above FSMAction will
            // set it incorrectly in this situation.
            stateBeforeAlarmMode = cd.clockData.stateBeforeAlarmMode;
        }
        if (stateMachinePtr->currentState == DISPLAY_ALARM) {
            actionToggleAlarmIndicator();
        }
    } 
    // Otherwise the data was invalid, so we will reset the system to a known state.
    // We will just set the clock and alarm default times, other settings have defaults configured for assignment.
    else {
        Clock_AddTime(&clock, 12, 0, 0);
        Clock_AddTime(&alarm, 7, 0, 0);
    }
}

void enableTimers() {
    // Primary millisecond counter
    enableTimer(TC2, TIMER2_CLOCK_SELECT_64_PRESCALER);
    // 7 Segment Display Update
    enableTimer(TC0, CLOCK_SELECT_1024_PRESCALER);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                //
//                             Hold increment/decrement functions/helpers                         //
//                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////


void incrementHoldTimerStart() {
    timerTaskIncrement->elaspedTime = timerTaskIncrement->period;
    TimerTaskEnable(timerTaskIncrement);
}
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

////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                //
//                                  Alarm Operations Functions                                    //
//                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////


void disableAlarmTimer() {
    // Disable the PWM timer.
    disableTimer(TC1);
    // Clear the count
    TCNT1  = 0;
    TCCR1A = 0;
}
void enableAlarmTimer() {
    TCCR1A = TC1_TCCR1A_CFG;
    enableTimer(TC1, CLOCK_SELECT_256_PRESCALER);
}
void alternateAlarm() { timerEnabled(TC1) ? disableAlarmTimer() : enableAlarmTimer(); }

void alarmActivate() {
    // Set the alarm reenable flag, this will be cleared by a timed task after 1 second, this
    // prevents the alarm from starting again if disabled within the first second of sounding.
    Flag_Set(&flag_alarmReenable);
    // Raise alarm sounding flag
    Flag_Set(&flag_alarmSounding);

    enableAlarmTimer();

    // Enable the timer tasks.
    TimerTaskEnable(timerTaskBuzzerDisable);
    TimerTaskEnable(timerTaskBuzzerPeriod);
    TimerTaskEnable(timerTaskAlarmReenableHold);
}

/**
 * @brief Deactivates the alarm.
 *  The alarm is silecned, sounding flag is cleared, and relevant timer tasks disabled/reset.
 */
void alarmDeactivate() {
    // Disable the PWM timer.
    disableTimer(TC1);
    // Clear the count
    TCNT1  = 0;
    TCCR1A = 0;
    // Disable and reset the buzzer period/alternating timer task.
    TimerTaskDisable(timerTaskBuzzerPeriod);
    TimerTaskReset(timerTaskBuzzerPeriod);

    // Clear/lower the alarm sounding flag.
    Flag_Clear(&flag_alarmSounding);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                //
//                             Finit State Machine Triggers                                       //
//                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

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

FSMTrigger shouldAlarmTrigger() { return flag_alarmEnabled.set && (Clock_CompareClocks(&clock, &alarm) == EQUAL) && !flag_alarmSounding.set && !flag_alarmReenable.set; }

FSMTrigger clearSoundingAlarm() { return flag_alarmSounding.set && incrementPressed(); }


////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                //
//                             Finit State Machine Actions                                        //
//                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

FSMAction actionIncrementHour() { Clock_AddHours(&clock, 1); }

FSMAction actionDecrementHour() { Clock_AddHours(&clock, -1); }

FSMAction actionIncrementMinute() { Clock_AddMinutes(&clock, 1); }

FSMAction actionDecrementMinute() { Clock_AddMinutes(&clock, -1); }

FSMAction actionIncrementSecond() { Clock_AddSeconds(&clock, 1); }

FSMAction actionIncrementHourAlarm() { Clock_AddHours(&alarm, 1); }

FSMAction actionDecrementHourAlarm() { Clock_AddHours(&alarm, -1); }

FSMAction actionIncrementMinuteAlarm() { Clock_AddMinutes(&alarm, 1); }

FSMAction actionDecrementMinuteAlarm() { Clock_AddMinutes(&alarm, -1); }

FSMAction actionToggleTimeDisplayModes() { timeMode = (timeMode == TWELVE_HOUR_TIME ? TWENTY_FOUR_HOUR_TIME : TWELVE_HOUR_TIME); }

FSMAction actionEnableAlarm() {
    TimerTaskDisable(timerTaskBlinkAlarmLed);
    Flag_Set(&flag_alarmEnabled);
    LED_On(&ledAlarm);
}

FSMAction actionToggleAlarm() {
    Flag_Toggle(&flag_alarmEnabled);
    LED_Toggle(&ledAlarm);
}

FSMAction actionToggleAlarmIndicator() { LED_Toggle(&ledAlarmDisplayIndicator); }

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

////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                //
//                                      Display Functions                                         //
//                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

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

    if (hours == 0) {
        // If we are after midnight, after subtraction hours will be at 0, move to 12 for display.
        if (timeMode == TWELVE_HOUR_TIME) {
            hours = 12;
        } else {
            // If 24 hours time, and midnight, output 24.
            hours = 24;
        }
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

    Clock localClockCpy = *clk;

    uint8_t ml = localClockCpy.minutes % 10;
    uint8_t mh = localClockCpy.minutes / 10;

    // 12 hour time adjustment.
    uint8_t hours = determineHoursValueFor(localClockCpy.hours);

    uint8_t hl = hours % 10;
    uint8_t hh = hours / 10;

    displayData.data[SEG_FAR_RIGHT] = mapChar(ml); // Far Right
    displayData.data[SEG_RIGHT]     = mapChar(mh); // Centre Right
    displayData.data[SEG_LEFT]      = mapChar(hl); // Center Left
    displayData.data[SEG_FAR_LEFT]  = mapChar(hh); // Far Left

    // Decimal Point
    displayData.data[SEG_LEFT] &= (withDp ? mapChar(DP) : mapChar(BLANK));

    // PM Indicator: Active if showing 12 hours time and hours are > 12.
    displayData.data[SEG_FAR_RIGHT] &= (timeMode == TWELVE_HOUR_TIME && localClockCpy.hours > 12 ? mapChar(DP) : mapChar(BLANK));
}

void displayFunctionTimeHHMM() { displayClockHHMMOnSegments(&clock); }

void displayFunctionAlarmHHMM() { displayClockHHMMOnSegments(&alarm); }

void displayFunctionTimeMMSS() {
    Clock localClockCpy = clock;

    uint8_t sl = localClockCpy.seconds % 10;
    uint8_t sh = localClockCpy.seconds / 10;

    uint8_t ml = localClockCpy.minutes % 10;
    uint8_t mh = localClockCpy.minutes / 10;

    displayData.data[SEG_FAR_RIGHT] = mapChar(sl); // Far Right
    displayData.data[SEG_RIGHT]     = mapChar(sh); // Centre Right
    displayData.data[SEG_LEFT]      = mapChar(ml); // Center Left
    displayData.data[SEG_FAR_LEFT]  = mapChar(mh); // Far Left

    // Decimal Point
    // Must show constantly when in MMSS state. Currently does not.
    displayData.data[SEG_LEFT] &= mapChar(DP);

    // PM Indicator
    displayData.data[SEG_FAR_RIGHT] &= (timeMode == TWELVE_HOUR_TIME && localClockCpy.hours > 12 ? mapChar(DP) : mapChar(BLANK));
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

/**
 * @brief Lookup table for functions which format data for the 7 segment display.
 * 
 */
DisplayFunctionPointer displayFunctions[FSM_STATE_COUNT] = {displayFunctionTimeHHMM, displayFunctionTimeMMSS, displayFunctionSetTimeHH, displayFunctionSetTimeMM, displayFunctionAlarmHHMM, displayFunctionSetAlarmHH, displayFunctionSetAlarmMM};

////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                //
//                                      'Update' Functions                                        //
//                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

void updateDisplay() {
    // If the ADC reports a maxmimum value, turn the display off. Otherwise call the displayFunction, if set.
    if (ADC_VALUE >= DISPLAY_CUTOFF) {
        displayFunctionBlank();
    } else {
        if (displayFunction)
            displayFunction();
    }
    // Push the data to the display.
    SevenSegmentUpdate(displayData.data);
}

void updateTimers() {
    TimerTaskUpdate(1 * timer2InterruptCount);
    timer2InterruptCount = 0;
}

void updateEeprom() {
    ClockDataStruct data = {clock.hours, clock.minutes, alarm.hours, alarm.minutes, stateMachinePtr->currentState, flag_alarmEnabled.set, (timeMode == TWENTY_FOUR_HOUR_TIME ? 1 : 0), stateBeforeAlarmMode};
    EEPROM_SetClockDataStruct(data);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                //
//                                            MAIN                                                //
//                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////


int main() {
    // Initialise hardware and software timers/tasks.
    initialise();

      // Setup the state machine transition table.
    // Order in which transitions are inserted will determine their priority, this matters for certain transitions (especially silence alarm transitions).
    stateMachinePtr = &(FSM_TRANSITION_TABLE){
        DISPLAY_HH_MM,
        {   // Silence Alarm Transitions - insert first so they may override all other inputs because silencing the alarm takes priority
            displayHoursSilenceAlarm, alarmSetHHSilenceAlarm, alarmSetMMSilenceAlarm, displayMinutesSilenceAlarm, displayAlarmSilenceAlarm, timeSetHrSilenceAlarm, timeSetMinSilenceAlarm,
            // Transitions from HH:MM
            displayHoursToDisplayMinutes, displayHoursToSetAlarm, displayHoursToSetTime, displayHoursToggleTimeMode, displayHoursToggleAlarm,
            // Transitions from MM:SS
            displayMinutesToDisplayAlarm, displayAlarmToDisplayHHMM, displayMinutesToSetAlarm, displayMinutesToggleAlarm,
            // Transitions from Alarm Display
            displayAlarmToggleAlarm, displayAlarmToSetAlarm,
            // Transitions from Clock set hours
            timeSetHrToMin, timeSetHrIncrement, timeSetHrDecrement,
            // Transitions from Clock set minutes
            timeSetMinToDisplayHr, timeSetMinIncrement, timeSetMinDecrement,
            // Transitions from Alarm set hours
            alarmSetHHToAlarmSetMM, alarmSetHHIncrement, alarmSetHHDecrement,
            // Transitions from Alarm set minutes
            alarmSetMMToLastDisplayMode, alarmSetMMIncrement, alarmSetMMDecrement}
    };

    initialiseSystemState();

    // Enable timers now that we have done the rest of our configuration.
    enableTimers();

    // Enable interrupts.
    sei();

    while (1) {
        // Update timed tasks.
        Flag_RunIfSet(&flag_updateTimers);

        // Update the FSM, and upon a state change clear all button input flags.
        if (FSMUpdate(stateMachinePtr) == STATE_CHANGE) {
            ButtonClearAllActionFlags();
        }

        // Assign the display function using the look-up table.
        displayFunction = displayFunctions[stateMachinePtr->currentState];

        // Update the 7 segment display.
        // Too fast to be use a TimerTask
        Flag_RunIfSet(&flag_updateDisplay);

        // Check if our alarm should be triggered, and if it should be, activate it.
        if (shouldAlarmTrigger()) {
            alarmActivate();
        }
    }

    return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                //
//                              Interrupt Service Routines                                        //
//                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

// Timer 0 Compare A - Timer 0 used for controlling 7 segment display brightness.
ISR(TIMER0_COMPA_vect) {
    // Set update display flag to signal to main loop to push out fresh display data.
    Flag_Set(&flag_updateDisplay);
}

// Main Counter
ISR(TIMER2_COMPA_vect) {
    // Add 1 millisecond to the system counter. Matter of priority so do that straight away.
    totalMillisecondsElasped++;
    timer2InterruptCount++;
    Flag_Set(&flag_updateTimers);
}

// ADC - Conversion Complete
ISR(ADC_vect) {
    // Set Timer 0 compare values to the value of ADCH.
    // Ignoring bottom two bits to avoid noise.
    OCR0A = ADCH;
}

// PCINT1 - Button Interrupts
ISR(PCINT1_vect) { ButtonUpdateAll(); }