#include "avr/interrupt.h"
#include "avr/io.h"
#include "bool.h"
#include "drivers/i2c/display/22S1_ELEC3042_I2C_PCF8574.h"
#include "drivers/mcp23s17/mcp23s17.h"
#include "flag.h"
#include "intersection/light.h"
#include "intersection/trafficLight.h"
#include "intersection/intersection.h"
#include "null.h"
#include "stdint.h"
#include "timerTask.h"
#include "timers.h"
#include "types.h"
#include "sensor.h"
#include "systemtimer.h"
#include "adc.h"
#include "fsm.h"
#include "period.h"

/*
    Broadway                ->  BWY
    Broadway South          ->  BWS
    Broadway Turn and Ped   ->  BTP
    Little Street           ->  LIT
    Broadway and Little Ped ->  BWP
*/
const char phaseStrings[][3] = {"BWY", "BWS", "BTP", "LIT", "BWP", "amb", "red"};

#define FREQ_TO_OCR(f) (F_CPU / (2 * f))

typedef struct {
    uint8_t minTime : 4;
    uint8_t minTimeShoulder : 4;
    uint8_t maxTime : 4;
    uint8_t maxTimeShoulder : 4;
} PhaseTimes;

const PhaseTimes phaseTimes[] = {
    (PhaseTimes){8, 0, 10, 0},   // Broadway
    (PhaseTimes){2, 0,  4, 0},   // Broadway Southbound
    (PhaseTimes){7, 3,  7, 3},   // Broadway Turn & Pedestrians
    (PhaseTimes){3, 0,  5, 0},   // Little Street
    (PhaseTimes){5, 3,  5, 3},   // Broadway Straight & little street pedestrians
    (PhaseTimes){3, 0,  3, 0},   // Amber / Yellow Lights
    (PhaseTimes){3, 0,  3, 0}    // Red Lights
};
uint8_t period_elasped_in_current_state = 0;

extern volatile uint64_t totalMillisecondsElasped;

extern FSM_STATE state_before_amber;
extern FSM_STATE state_after_red;

IntersectionLightState intersectionStates[MAX_STATES];

TrafficLight tl_Broadway_North;
TrafficLight tl_Broadway_North_Turn;
TrafficLight tl_Broadway_South;
TrafficLight tl_Broadway_South_Turn;
TrafficLight tl_Broadway_Pedestrian;
TrafficLight tl_Little_Street_Pedestrian;
TrafficLight tl_Little_Street;

volatile uint16_t periodCounter = 0;

TimerTask *tt_period;
TimerTask *tt_updateDisplay;
TimerTask *tt_hazardCycle;
TimerTask *tt_hazardCancel;
TimerTask *tt_pedestrianFlash;

/**
 * Initialiser Prototypes
 */
Initialiser initialise();
Initialiser initialiseTimer2();
Initialiser initialiseTimer1();
Initialiser initialiseFlags();
Initialiser initialiseMCP23S17();
Initialiser initialiseTrafficLights();
Initialiser initialiseTimerTasks();
Initialiser initialiseInterrupt0();

Action actionUpdateStatusDisplay();
Action actionCheckMcp23s17ButtonInterrupts();
Action actionCheck328Buttons();
Action actionUpdateIntersection();

void enableTimers();
void updateTimers();

void flashPedestrianLight();

const uint8_t LCD_Addr = 0x27;

void ttPeriodElasped();

void pwrSave();

void clearAllLights();

/**
 * Flag declarations
 */

Flag flag_UpdateTimers;
Flag flag_UpdateDisplay;
Flag flag_CheckExternalButtonInterrupts;
Flag flag_CheckInternalButtonInterrupts;
Flag flag_UpdateIntersection;

Sensor sensor0 = {0, RELEASED, RELEASED};
Sensor sensor1 = {0, RELEASED, RELEASED};
Sensor sensor2 = {0, RELEASED, RELEASED};
Sensor sensor3 = {0, RELEASED, RELEASED};
Sensor sensor4 = {0, RELEASED, RELEASED};
Sensor sensor5 = {0, RELEASED, RELEASED};
Sensor sensor6 = {0, RELEASED, RELEASED};

char sensorStateChar(Sensor *b) { return b->triggered ? 'X' : 'O'; }

typedef enum { SS_HAZARD, SS_NORMAL } SystemState;

#define IGNORE_HAZARD_STARTUP

#ifdef IGNORE_HAZARD_STARTUP
SystemState ss = SS_NORMAL;
#else
SystemState ss = SS_HAZARD;
#endif

/*
    Keeps track of how many times the milliseconds timer/counter (TCC2) is triggered.
    In the event the system misses the chance to updated timed/scheduled tasks we can use this value
    to add 1 (ms) * timer2InterruptCount to determine the delta since tasks where last updated.
    This value will be reset to zero after updating tasks.
 */
uint8_t timer2InterruptCount = 0;

Initialiser initialise() {
    initialiseTimer2();
    initialiseTimer1();
    initialiseADC();

    setup_I2C();
    setup_LCD(LCD_Addr);

    initialiseInterrupt0();

    initialiseFlags();
    Flag_Set(&flag_UpdateDisplay);

    initialiseMCP23S17();

    initialiseTrafficLights();
    initialiseTimerTasks();

    // Internal LED Broadway Pedestrian
    DDRD |= (1 << DDD6) | (1 << DDD5);
    // Internal LED Little Street Pedestrian
    DDRC |= (1 << DDC2) | (1 << DDC3) | (1 << DDC1);
    PORTC |= (1 << PC1);

    // Hazard Pull-up + S3 and S5
    DDRD &= ~((1 << DDD3) | (1 << DDD4) | (1 << DDD7));
    PORTD |= (1 << PD3) | (1 << PD4) | (1 << PD7);

    // Broadway North Turn
    DDRB |= (1 << DDB0);

    // 328p button interrupts
    PCMSK2 |= (1 << PCINT20) | (1 << PCINT23) | (1 << PCINT19); // S3, S5 & Hazard
    PCMSK1 |= (1 << PCINT9); // S6 Interrupt
    PCICR |= (1 << PCIE1) | (1 << PCIE2);

    initialiseIntersectionStates();

    ADC_StartConversion();
}

Initialiser initialiseMCP23S17() {

    // Initialise MCP23S17, initialiser handles SPI configuration.
    MCP23S17_Initialise();

    // Initialise Port A
    MCP23S17_IoDirectionSetRegister(MCP_PORT_A, 0b00010001);
    MCP23S17_PullUpSetRegister(MCP_PORT_A, (1 << PU0) | (1 << PU4)); // Enable Pull-Up Resistors on buttons
    MCP23S17_InterruptEnable(MCP_PORT_A, GPINT0);
    MCP23S17_InterruptEnable(MCP_PORT_A, GPINT4);

    // Initialise Port B
    MCP23S17_IoDirectionSetRegister(MCP_PORT_B, 0b00010001);
    MCP23S17_PullUpSetRegister(MCP_PORT_B, (1 << PU0) | (1 << PU4)); // Enable Pull-Up Resistors on buttons
    MCP23S17_InterruptEnable(MCP_PORT_B, GPINT0);
    MCP23S17_InterruptEnable(MCP_PORT_B, GPINT4);
}

Initialiser initialiseFlags() {
    flag_UpdateDisplay                 = Flag_Create(&actionUpdateStatusDisplay, NULL);
    flag_CheckExternalButtonInterrupts = Flag_Create(&actionCheckMcp23s17ButtonInterrupts, NULL);
    flag_CheckInternalButtonInterrupts = Flag_Create(&actionCheck328Buttons, NULL);
    flag_UpdateIntersection            = Flag_Create(&actionUpdateIntersection, NULL);
    Flag_Set(&flag_UpdateIntersection);
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

// Speaker tone generator
Initialiser initialiseTimer1() {
    disableTimer(TC1);
    // Timer/Counter Control Register A (Compare Output Modes and Waveform
    TCCR1A = (1 << COM1A0); // Waveform Generation Mode - Fast PWM, Mode 7
    //  Timer/Counter Control Register B (Input Capture Noise Canceler, Input
    //  Capture Edge Select, Waveform Generation Mode (bits 13 and 12), and Clock
    //  Select)
    TCCR1B = (1 << WGM12); //CTC Mode 4

    // Timer/Counter Control Register C (Force Output Compare)
    TCCR1C = 0x00;

    // Timer/Counter Value
    TCNT1 = 0;

    // Input Capture Register
    //ICR1 = 250000 / 277 - 1; //250000 = f_cpu / prescaler -> 16_000_000 / 64
    ICR1 = 0;

    // Output Compare Register A
    OCR1A = 0;
    // Output Compare Register B
    OCR1B = 0;

    // Timer/Counter Interrupt Mask Register (Input Capture Interrupt Enable,
    // Output Compare A/B Match Interrupt Enable, and Overflow Interrupt Enable)
    TIMSK1 = 0;

    // Timer/Counter Interrupt Flag Register (Input Capture, Output Compare A/B
    // and Overflow Flags)
    TIFR1 = 0x00; // Ensure all timer interrupt flags are cleared.

    // Enable output on pin PB1/pin 9
    DDRB |= (1 << DDB1);
    PORTB &= ~(1 << PORTB1);
}

Initialiser initialiseTrafficLights() {
    tl_Broadway_North = (TrafficLight){
        (Light){External, (ExternalLight){MCP_PORT_B, GP5}, {NULL}},
        (Light){External, (ExternalLight){MCP_PORT_B, GP6}, {NULL}},
        (Light){External, (ExternalLight){MCP_PORT_B, GP7}, {NULL}}
    };
     tl_Broadway_North_Turn = (TrafficLight){
        (Light){Internal, {NULL}, (InternalLight){&PORTB, PB0}},
        (Light){-1, {NULL}, {NULL}},
        (Light){-1, {NULL}, {NULL}}
    };
    tl_Broadway_South = (TrafficLight){
        (Light){External, (ExternalLight){MCP_PORT_A, GP1}, {NULL}},
        (Light){External, (ExternalLight){MCP_PORT_A, GP2}, {NULL}},
        (Light){External, (ExternalLight){MCP_PORT_A, GP3}, {NULL}}
    };
    tl_Broadway_South_Turn = (TrafficLight){
        (Light){External, (ExternalLight){MCP_PORT_A, GP5}, {NULL}},
        (Light){External, (ExternalLight){MCP_PORT_A, GP6}, {NULL}},
        (Light){External, (ExternalLight){MCP_PORT_A, GP7}, {NULL}}
    };
    tl_Little_Street = (TrafficLight){
        (Light){External, (ExternalLight){MCP_PORT_B, GP1}, {NULL}},
        (Light){External, (ExternalLight){MCP_PORT_B, GP2}, {NULL}},
        (Light){External, (ExternalLight){MCP_PORT_B, GP3}, {NULL}}
    };
    tl_Broadway_Pedestrian = (TrafficLight){
        (Light){Internal, {NULL}, (InternalLight){&PORTD, PD5}},
        (Light){Internal, {NULL}, (InternalLight){&PORTD, PD5}}, // Duplicate red light to yellow so it becomes our hazard light too.
        (Light){Internal, {NULL}, (InternalLight){&PORTD, PD6}},
    };
    tl_Little_Street_Pedestrian = (TrafficLight){
        (Light){Internal, {NULL}, (InternalLight){&PORTC, PC2}},
        (Light){Internal, {NULL}, (InternalLight){&PORTC, PC2}},
        (Light){Internal, {NULL}, (InternalLight){&PORTC, PC3}}
    };
}

Initialiser initialiseInterrupt0() {
    // Enable interrupts on INT0
    EIMSK = (1 << INT0);
    // Respond to any logic level change on INT0
    EICRA = (0 << ISC01) | (1 << ISC00);
}

void hazardState() {
    TrafficLight_Hazard(&tl_Broadway_North);
    TrafficLight_Hazard(&tl_Broadway_North_Turn);
    TrafficLight_Hazard(&tl_Broadway_South);
    TrafficLight_Hazard(&tl_Broadway_South_Turn);
    TrafficLight_Hazard(&tl_Little_Street);
    TrafficLight_Hazard(&tl_Broadway_Pedestrian);
    TrafficLight_Hazard(&tl_Little_Street_Pedestrian);
}

void endHazardState() {
    ss = SS_NORMAL;
    TimerTaskDisable(tt_hazardCycle);
    //After we end hazard state return to BROADWAY intersection state.
    transitionTable.currentState = BROADWAY; 
}

TimerTask *tt_sound_idle_stop;
TimerTask *tt_sound_idle_start;

void pedestrian_idle_start() {
    enableTimer(TC1, TIMER1_CLOCK_SELECT_8_PRESCALER);
    OCR1A = FREQ_TO_OCR(973);

    TimerTaskRestart(tt_sound_idle_stop);
}

bool stopCalled = false;
void pedestrian_idle_stop() {
    disableTimer(TC1);
    TCNT1 = 0;

    stopCalled = !stopCalled;

    //TimerTaskRestart(tt_sound_idle_start);
}

// void pedestrian_idle() {
//     static bool running = false;
//     if (running) {
//         disableTimer(TC1);
//         TCNT1 = 0;
//         OCR1A = 0;
//         running = false;
//         // TimerTaskDisable(tt_sound_idle);
//     } else {
//         OCR1A               = FREQ_TO_OCR(973);
//         enableTimer(TC1, TIMER1_CLOCK_SELECT_8_PRESCALER);
//         running = true;
//     }
// }

// void pedestrian_play_idle() {
//     // TimerTaskReset(tt_sound_idle);
//     // TimerTaskEnable(tt_sound_idle);
// }

// int tones[] = {
//   3465, 2850, 2333, 1956, 1638, 1380, 1161, 992, 814, 704, 500
// }; 

// void playChirpTask() {
//     static uint8_t i = 0;
//     OCR1A    = tones[i++];
//     if (i >= 11) {
//         i = 0;
//         TCNT1 = 0;
//         disableTimer(TC1);
//         // TimerTaskDisable(tt_chirp);
//     }
// }

// void chirpInit() {
//     // TimerTaskReset(tt_chirp);
//     // TimerTaskEnable(tt_chirp);
//     enableTimer(TC1, TIMER1_CLOCK_SELECT_8_PRESCALER);
// }

// void playIdle() {
//     static bool playing = false;
//     if (playing) {
//         disableTimer(TC1);
//         TCNT1 = 0;
//     }
//     enableTimer(TC1, TIMER1_CLOCK_SELECT_8_PRESCALER);
//     OCR1A = FREQ_TO_OCR(973);
// }

Initialiser initialiseTimerTasks() {
    tt_period        = TimerTaskCreate(PERIOD_MS, &ttPeriodElasped, NULL, true, false);
    
    tt_hazardCycle   = TimerTaskCreate(1000L, &hazardState, NULL, true, false);
    #ifdef IGNORE_HAZARD_STARTUP
        TimerTaskDisable(tt_hazardCycle);
    #endif

    tt_updateDisplay = TimerTaskCreate(100L, &Flag_Set, &flag_UpdateDisplay, true, false);
    tt_hazardCancel  = TimerTaskCreate(10000L, &endHazardState, NULL, false, true);

    tt_pedestrianFlash = TimerTaskCreate(PERIOD_MS / 2, &flashPedestrianLight, NULL, false, false);

    //tt_sound_idle_start = TimerTaskCreate(1800L, pedestrian_idle_start, NULL, true, true);
    tt_sound_idle_stop = TimerTaskCreate(100L, pedestrian_idle_stop, NULL, false, true);
}

void enableTimers() {
    // Primary millisecond counter
    enableTimer(TC2, TIMER2_CLOCK_SELECT_64_PRESCALER);
    //enableTimer(TC1, TIMER1_CLOCK_SELECT_8_PRESCALER);
}

void updateTimers() {
    TimerTaskUpdate(1 * timer2InterruptCount);
    timer2InterruptCount = 0;
}

void internalButtonHeldTime(Sensor *sensor) {
    if (!sensor)
        return;

    if (sensor->state == PRESSED) {
        sensor->periods_held++;
    }
}

void handleHeldInputs() {
    internalButtonHeldTime(&sensor0);
    internalButtonHeldTime(&sensor1);
    internalButtonHeldTime(&sensor2);
    internalButtonHeldTime(&sensor3);
    internalButtonHeldTime(&sensor4);
    internalButtonHeldTime(&sensor5);
    internalButtonHeldTime(&sensor6);
}

Sensor intersection_change_trigger_sensor;

uint8_t getStatePeriodChangeTime() {
    const PhaseTimes STATE_PERIODS = phaseTimes[transitionTable.currentState];

    if (intersection_change_trigger_sensor.periods_held >= STATE_PERIODS.maxTime + STATE_PERIODS.maxTimeShoulder)
        return STATE_PERIODS.maxTime + STATE_PERIODS.maxTimeShoulder;                           // Return calculated maximum time
    else if (intersection_change_trigger_sensor.periods_held < STATE_PERIODS.minTime + STATE_PERIODS.minTimeShoulder)
        return STATE_PERIODS.minTime + STATE_PERIODS.minTimeShoulder;                           // Return calculated minimum time
    else 
        return intersection_change_trigger_sensor.periods_held; // Return held time
}

void flashPedestrianLight() {
    switch (transitionTable.currentState) {
        case BROADWAY_TURN_AND_PEDESTRIANS:
            TrafficLight_Toggle(&tl_Broadway_Pedestrian.green);
            break;
        case BROADWAY_STRAIGHT_AND_LITTLE_STREET_PEDESTRIAN:
            TrafficLight_Toggle(&tl_Little_Street_Pedestrian.green);
            break;
        default:
            return;
    }
}

uint8_t state_change_time;
uint8_t chirpPeriodCount = 0;
char stateType           = 'R';
void ttPeriodElasped() {
    periodCounter++; 
    if (periodCounter > 99999)
        periodCounter = 0;

    // Handle held buttons
    handleHeldInputs();


    stateType = (transitionTable.currentState == BROADWAY_TURN_AND_PEDESTRIANS || transitionTable.currentState == BROADWAY_STRAIGHT_AND_LITTLE_STREET_PEDESTRIAN) ? 'P' : 'C';
    // Advance intersection state logic
    if (ss == SS_NORMAL) {
        state_change_time = getStatePeriodChangeTime();

        const PhaseTimes STATE_PERIODS = phaseTimes[transitionTable.currentState];
        // Handle pedestrian flashing. 
        // If current periods equals max (or min) time and the shoulder time is not 0, activate flash task.
        if ((period_elasped_in_current_state >= STATE_PERIODS.maxTime && STATE_PERIODS.maxTimeShoulder != 0) ||
            (period_elasped_in_current_state >= STATE_PERIODS.minTime && STATE_PERIODS.minTime != 0)) {
                // If not enabled, reset and enable
            if (!tt_pedestrianFlash->enabled) {
                TimerTaskReset(tt_pedestrianFlash);
                TimerTaskEnable(tt_pedestrianFlash);
            }
        }

        if (period_elasped_in_current_state >= state_change_time) {
            Flag_Set(&flag_UpdateIntersection);
            if (tt_pedestrianFlash->enabled) {
                TimerTaskDisable(tt_pedestrianFlash);
            }
        }

        const bool isPedestrianState = (transitionTable.currentState == BROADWAY_TURN_AND_PEDESTRIANS || transitionTable.currentState == BROADWAY_STRAIGHT_AND_LITTLE_STREET_PEDESTRIAN);
        if (isPedestrianState) {
            pedestrian_idle_stop();
        } else {
            chirpPeriodCount++;

            if (chirpPeriodCount % 5 == 0) {
                pedestrian_idle_start();
            }

            if (chirpPeriodCount == 255) {// Roll over now to avoid 2 quick chirps
                chirpPeriodCount = 0;
            }
        }
    }

    period_elasped_in_current_state++;
}

int main(void) {
    initialise();

    flag_UpdateTimers = Flag_Create(&updateTimers, NULL);

    // Enable timers now that we have done the rest of our configuration.
    enableTimers();

    // Enable interrupts.
    sei();

    while (1) {
        // Handles Hazard / Normal Operation States
        // No hazard signal
        if (PIND & (1 << PIND3)) {
            if (!tt_hazardCancel->enabled && ss == SS_HAZARD) {
                TimerTaskReset(tt_hazardCancel);
                TimerTaskEnable(tt_hazardCancel);
            }

            Flag_RunIfSet(&flag_UpdateIntersection);

        } else { // Hazard signal active
            if (ss == SS_NORMAL) {
                ss = SS_HAZARD;
                clearAllLights();
                TimerTaskEnable(tt_hazardCycle);
                TimerTaskDisable(tt_hazardCancel);
            }
        }

        // Update timed tasks.
        Flag_RunIfSet(&flag_UpdateTimers);

        // Update Display
        Flag_RunIfSet(&flag_UpdateDisplay);

        // Check MCP23S17 Interrupts
        Flag_RunIfSet(&flag_CheckExternalButtonInterrupts);
        // Check 328P Interrupts
        Flag_RunIfSet(&flag_CheckInternalButtonInterrupts);

        TimerTaskSetPeriod(tt_period, PERIOD_MS);
        TimerTaskSetPeriod(tt_pedestrianFlash, PERIOD_MS / 2);

        // Testing Speaker/Buzzer
        //OCR1A = FREQ_TO_OCR(500);
        //OCR1A = adc_value << 4;
    }

    return 0;
}

Action actionUpdateIntersection() {
    // Update the intersection FSM.
    const FSMUpdateResult HAS_STATE_CHANGED = FSMUpdate(&transitionTable);

    // If the state did chance, reset periods elasped in current state.
    if (HAS_STATE_CHANGED == STATE_CHANGE)
        period_elasped_in_current_state = 0;

    if (ss == SS_NORMAL) {
        // References to current state and next intersection states (excluding amber and red)
        IntersectionLightState *currentIntersectionState = &intersectionStates[state_before_amber];
        IntersectionLightState *nextIntersectionState = &intersectionStates[state_after_red];

        // If currently amber, mix states with yellow
        if (transitionTable.currentState == INTERSECTION_AMBER) {
            IntersectionLightState mixedState = mixIntersectionStates(currentIntersectionState, nextIntersectionState, YELLOW);
            applyIntersectionState(&mixedState);
        } 
        // If currently red, mix states with red
        else if (transitionTable.currentState == INTERSECTION_RED) {
            IntersectionLightState mixedState = mixIntersectionStates(currentIntersectionState, nextIntersectionState, RED);
            applyIntersectionState(&mixedState);
        } 
        // Otherwise just apply the current intersection state.
        else {
            applyIntersectionState(&intersectionStates[transitionTable.currentState]);
        }
    }
}

// Break these out into individual functions, then when a value is updated
// we can just set the cursor accordingly and update said character, not
// having to bother about the entire display, want to save as much time
// as possible by avoid i2c transfers.
//
// Batch all the data into rows to limit i2c calls potentially?

Action actionUpdateStatusDisplay() {
    // Row row initially all spaces.
    char row[16] = {0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20};
    row[0]       = sensorStateChar(&sensor0);
    row[1]       = sensorStateChar(&sensor1);
    row[2]       = sensorStateChar(&sensor2);
    row[3]       = sensorStateChar(&sensor3);
    row[4]       = sensorStateChar(&sensor4);
    row[5]       = sensorStateChar(&sensor5);
    row[6]       = sensorStateChar(&sensor6);

    uint16_t period = periodCounter;
    for (int i = 4; i >= 0; i--) {
        row[11 + i] = (period % 10) + 48;
        period /= 10;
    }

    // Intersection change trigger sensor
    // row[8] = 'P';
    // row[9] = 48 + intersection_change_trigger_sensor.periods_held;

    LCD_Position(LCD_Addr, 0x0);     // Top left first character position
    LCD_Write(LCD_Addr, row, 16); // Write top row

    // Clear row (Set to spaces)
    for (int i = 3; i < sizeof(row); i++) {
        row[i] = 0x20;
    }

    if (ss == SS_HAZARD) {
        row[0] = 'H';
        row[1] = 'A';
        row[2] = 'Z';
    } else {
        const char *phaseStr = phaseStrings[transitionTable.currentState];
        row[3] = 'G';
        if (transitionTable.currentState == INTERSECTION_AMBER) {
            phaseStr = phaseStrings[state_before_amber];
            row[3]   = 'Y';
        } else if (transitionTable.currentState == INTERSECTION_RED) {
            phaseStr = phaseStrings[state_before_amber];
            row[3] = 'R';
        }

        for (int i = 0; i <= 2; i++) {
            row[i] = phaseStr[i];
        }
    }

    // Period Count
    // uint16_t currentPeriods = period_elasped_in_current_state;
    // for (int i = 1; i >= 0; i--) {
    //     row[5 + i] = (currentPeriods % 10) + 48;
    //     currentPeriods /= 10;
    // }

    // ADC Period LUT Value
    // uint16_t period_ms = PERIOD_MS;
    // for (int i = 3; i >= 0; i--) {
    //     row[12 + i] = (period_ms % 10) + 48;
    //     period_ms /= 10;
    // }

    // uint16_t speakerValue = OCR1A;
    // for (int i = 4; i >= 0; i--) {
    //     row[10 + i] = (speakerValue % 10) + 48;
    //     speakerValue /= 10;
    // }

    // row[8] = 'E';
    // row[9] = 48 + tt_sound_idle_start->enabled;
    // row[11] = 'S';
    // row[12] = 48 + tt_sound_idle_stop->enabled;

    // row[14] = 48 + stopCalled;

    // row[9] = state_change_time + 48;

    // uint8_t chirp = chirpPeriodCount;
    // for (int i = 4; i >= 0; i--) {
    //     row[10 + i] = (chirp % 10) + 48;
    //     chirp /= 10;
    // }

    // row[8]  = 48 + (transitionTable.currentState == BROADWAY_TURN_AND_PEDESTRIANS || transitionTable.currentState == BROADWAY_STRAIGHT_AND_LITTLE_STREET_PEDESTRIAN);
    // row[10] = 48 + transitionTable.currentState;
    // row[12] = stateType;

    // row[11] = intersection_change_trigger_sensor.periods_held + 48;

    LCD_Position(LCD_Addr, 0x40);             // Bottom left first character position
    //LCD_Write(LCD_Addr, &phaseStrings[phase], 3); // Write System Phase.
    //LCD_Write(LCD_Addr, "Y ", 2);                  // Write phase color
    //LCD_Write(LCD_Addr, &portAFlagsDisplay, 8);
    LCD_Write(LCD_Addr, row, 16); // Write top row
}

Action actionCheckMcp23s17ButtonInterrupts() {
    uint8_t portAFlags = MCP23S17_InterruptFlagReadRegister(MCP_PORT_A);

    if (portAFlags) {
        uint8_t portAChangeBits = MCP23S17_InterruptCaptureReadRegister(MCP_PORT_A);
         // Sensor 1
        Sensor_CheckState_External(portAFlags, portAChangeBits, ICP4, &sensor1);

        // Sensor 0
        Sensor_CheckState_External(portAFlags, portAChangeBits, ICP0, &sensor0);
    }

    uint8_t portBFlags = MCP23S17_InterruptFlagReadRegister(MCP_PORT_B);

    if (portBFlags) {
        uint8_t portBChangeBits = MCP23S17_InterruptCaptureReadRegister(MCP_PORT_B);
        // Sensor 2
        Sensor_CheckState_External(portBFlags, portBChangeBits, ICP0, &sensor2);

        // Sensor 4
        Sensor_CheckState_External(portBFlags, portBChangeBits, ICP4, &sensor4);
    }
}

Action actionCheck328Buttons() {
    Sensor_CheckState_Internal(&PIND, PD7, &sensor3);
    Sensor_CheckState_Internal(&PIND, PD4, &sensor5);
    Sensor_CheckState_Internal(&PINC, PC1, &sensor6);
}

void clearAllLights() {
    TrafficLight_Blank(&tl_Broadway_North);
    TrafficLight_Blank(&tl_Broadway_North_Turn);
    TrafficLight_Blank(&tl_Broadway_South);
    TrafficLight_Blank(&tl_Broadway_South_Turn);
    TrafficLight_Blank(&tl_Little_Street);
    TrafficLight_Blank(&tl_Broadway_Pedestrian);
    TrafficLight_Blank(&tl_Little_Street_Pedestrian);
}

// INT0 ISR used to handle interrupts from MCP23S17.
ISR(INT0_vect) {
    Flag_Set(&flag_CheckExternalButtonInterrupts);
}

// PCINT1 used for S6 interrupts
ISR(PCINT1_vect) { Flag_Set(&flag_CheckInternalButtonInterrupts); }

// PCINT1 used for S3, s5 and hazard interrupts
ISR(PCINT2_vect) { Flag_Set(&flag_CheckInternalButtonInterrupts); }

// Main Counter - 1 ms resolution
ISR(TIMER2_COMPA_vect) {
    // Add 1 millisecond to the system counter. Matter of priority so do that straight away.
    totalMillisecondsElasped++;
    ++timer2InterruptCount;
    Flag_Set(&flag_UpdateTimers);
}