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

// ADV value to Period Time MS Lookup table.
const uint16_t ADC_TO_PERIOD_TIME_MS_LUT[] = {80, 83, 87, 90, 94, 98, 101, 105, 108, 112, 116, 119, 123, 126, 130, 134, 137, 141, 144, 148, 152, 155, 159, 162, 166, 170, 173, 177, 181, 184, 188, 191, 195, 199, 202, 206, 209, 213, 217, 220, 224, 227, 231, 235, 238, 242, 245, 249, 253, 256, 260, 264, 267, 271, 274, 278, 282, 285, 289, 292, 296, 300, 303, 307, 310, 314, 318, 321, 325, 328, 332, 336, 339, 343, 346, 350, 354, 357, 361, 365, 368, 372, 375, 379, 383, 386, 390, 393, 397, 401, 404, 408, 411, 415, 419, 422, 426, 429, 433, 437, 440, 444, 448, 451, 455, 458, 462, 466, 469, 473, 476, 480, 484, 487, 491, 494, 498, 502, 505, 509, 512, 516, 520, 523, 527, 530, 534, 538, 541, 545, 549, 552, 556, 559, 563, 567, 570, 574, 577, 581, 585, 588, 592, 595, 599, 603, 606, 610, 613, 617, 621, 624, 628, 632, 635, 639, 642, 646, 650, 653, 657, 660, 664, 668, 671, 675, 678, 682, 686, 689, 693, 696, 700, 704, 707, 711, 714, 718, 722, 725, 729, 733, 736, 740, 743, 747, 751, 754, 758, 761, 765, 769, 772, 776, 779, 783, 787, 790, 794, 797, 801, 805, 808, 812, 816, 819, 823, 826, 830, 834, 837, 841, 844, 848, 852, 855, 859, 862, 866, 870, 873, 877, 880, 884, 888, 891, 895, 898, 902, 906, 909, 913, 917, 920, 924, 927, 931, 935, 938, 942, 945, 949, 953, 956, 960, 963, 967, 971, 974, 978, 981, 985, 989, 992, 996, 1000};
/**
 * 
 * Average ADC over 10 samples.
 * 
 */


/// PROTOTYPES
/*
    Broadway                ->  BWY
    Broadway South          ->  BWS
    Broadway Turn and Ped   ->  BTP
    Little Street           ->  LIT
    Broadway and Little Ped ->  BWP
*/
const char phaseStrings[][3] = {"BWY", "BWS", "BTP", "LIT", "BWP", "amb", "red"};

typedef struct {
    uint8_t minTime : 4;
    uint8_t minTimeShoulder : 4;
    uint8_t maxTime : 4;
    uint8_t maxTimeShoulder : 4;
} PhaseTimes;

const PhaseTimes phaseTimes[5] = {
    (PhaseTimes){8, 0, 10, 0}, // Broadway
    (PhaseTimes){2, 0, 4, 0}, // Broadway South
    (PhaseTimes){7, 3, 7, 3},// Broadway  Turn & Pedestrians

};

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
TimerTask *tt_sampleAdc;
/// ^^^^^ PROTOTYPES

/**
 * Initialiser Prototypes
 */
Initialiser initialise();
Initialiser initialiseTimer2();
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

const uint8_t LCD_Addr = 0x27;

void ttPeriodElasped();

void pwrSave();
/**
 * Flag declarations
 */

Flag flag_UpdateTimers;
Flag flag_UpdateDisplay;
Flag flag_CheckExternalButtonInterrupts;
Flag flag_CheckInternalButtonInterrupts;
Flag flag_UpdateIntersection;

Sensor sensor0 = {0, RELEASED};
Sensor sensor1 = {0, RELEASED};
Sensor sensor2 = {0, RELEASED};
Sensor sensor3 = {0, RELEASED};
Sensor sensor4 = {0, RELEASED};
Sensor sensor5 = {0, RELEASED};
Sensor sensor6 = {0, RELEASED};
Sensor sensorHazard = {0, RELEASED};

char sensorStateChar(Sensor *b) { return b->triggered ? 'X' : 'O'; }

typedef enum { SS_HAZARD, SS_NORMAL } SystemState;

#define IGNORE_HAZARD_STARTUP

#ifdef IGNORE_HAZARD_STARTUP
SystemState ss = SS_NORMAL; // SHOULD DEFAULT TO NORMAL
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

        // Hazard Interrupt Not really worth it is it?
        // PCMSK0 |= (1 << PC3); // D3/Pin 18
        // PCICR |= (1 << PCIE0);
        // Hazard Pull-up + S3 and S5
        DDRD &= ~((1 << DDD3) | (1 << DDD4) | (1 << DDD7));
    PORTD |= (1 << PD3) | (1 << PD4) | (1 << PD7);


    // 328p button interrupts
    PCMSK2 |= (1 << PCINT20) | (1 << PCINT23) | (1 << PCINT19); // S3, S5 & Hazard
    PCMSK1 |= (1 << PCINT9); // S6 Interrupt
    PCICR |= (1 << PCIE1) | (1 << PCIE2);

    initialiseIntersectionStates();
}

Initialiser initialiseMCP23S17() {

    // Initialise MCP23S17, initialiser handles SPI configuration.
    MCP23S17_Initialise();

    // Initialise Port A
    MCP23S17_IoDirectionSetRegister(MCP_PORT_A, 0b00010001);
    // MCP23S17_GpioSetRegister(MCP_PORT_A, 0b11101110);                // Turn all on
    MCP23S17_PullUpSetRegister(MCP_PORT_A, (1 << PU0) | (1 << PU4)); // Enable Pull-Up Resistors on buttons
    MCP23S17_InterruptEnable(MCP_PORT_A, GPINT0);
    MCP23S17_InterruptEnable(MCP_PORT_A, GPINT4);

    // Initialise Port B
    MCP23S17_IoDirectionSetRegister(MCP_PORT_B, 0b00010001);
    // MCP23S17_GpioSetRegister(MCP_PORT_B, 0b11101110);                // Turn all on
    MCP23S17_PullUpSetRegister(MCP_PORT_B, (1 << PU0) | (1 << PU4)); // Enable Pull-Up Resistors on buttons
    MCP23S17_InterruptEnable(MCP_PORT_B, GPINT0);
    MCP23S17_InterruptEnable(MCP_PORT_B, GPINT4);
}

Initialiser initialiseFlags() {
    flag_UpdateDisplay                 = Flag_Create(&actionUpdateStatusDisplay, NULL);
    flag_CheckExternalButtonInterrupts = Flag_Create(&actionCheckMcp23s17ButtonInterrupts, NULL);
    flag_CheckInternalButtonInterrupts = Flag_Create(&actionCheck328Buttons, NULL);
    flag_UpdateIntersection            = Flag_Create(&actionUpdateIntersection, NULL);
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

void clearAllLights() {
    TrafficLight_Blank(&tl_Broadway_North);
    TrafficLight_Blank(&tl_Broadway_North_Turn);
    TrafficLight_Blank(&tl_Broadway_South);
    TrafficLight_Blank(&tl_Broadway_South_Turn);
    TrafficLight_Blank(&tl_Little_Street);
    TrafficLight_Blank(&tl_Broadway_Pedestrian);
    TrafficLight_Blank(&tl_Little_Street_Pedestrian);
}

void endHazardState() {
    ss = SS_NORMAL;
    TimerTaskDisable(tt_hazardCycle);
    clearAllLights();
}

Initialiser initialiseTimerTasks() {
    tt_period        = TimerTaskCreate(2000L, &ttPeriodElasped, NULL, true, false);
    
    tt_hazardCycle   = TimerTaskCreate(1000L, &hazardState, NULL, true, false);
    #ifdef IGNORE_HAZARD_STARTUP
        TimerTaskDisable(tt_hazardCycle);
    #endif

    tt_updateDisplay = TimerTaskCreate(100L, &Flag_Set, &flag_UpdateDisplay, true, false);
    tt_hazardCancel  = TimerTaskCreate(10000L, &endHazardState, NULL, false, true);
    tt_sampleAdc     = TimerTaskCreate(250L, &ADC_StartConversion, NULL, true, false);
}

void enableTimers() {
    // Primary millisecond counter
    enableTimer(TC2, TIMER2_CLOCK_SELECT_64_PRESCALER);
}

void updateTimers() {
    TimerTaskUpdate(1 * timer2InterruptCount);
    timer2InterruptCount = 0;
}

void ttPeriodElasped() { 
    periodCounter++; 
    if (periodCounter > 99999)
        periodCounter = 0;

    if (ss == SS_NORMAL)
        Flag_Set(&flag_UpdateIntersection);
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
    }

    return 0;
}



uint8_t fsmSwitches = 0;
Action actionUpdateIntersection() {
    FSMUpdate(&transitionTable);
    fsmSwitches++;

    if (ss == SS_NORMAL) {
        IntersectionLightState *currentIntersectionState = &intersectionStates[state_before_amber];
        IntersectionLightState *nextIntersectionState = &intersectionStates[state_after_red];

        if (transitionTable.currentState == INTERSECTION_AMBER) {
            IntersectionLightState mixedState = mixIntersectionStates(currentIntersectionState, nextIntersectionState, YELLOW);
            applyIntersectionState(&mixedState);
        } else if (transitionTable.currentState == INTERSECTION_RED) {
            IntersectionLightState mixedState = mixIntersectionStates(currentIntersectionState, nextIntersectionState, RED);
            applyIntersectionState(&mixedState);
        } else {
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
        for (int i = 0; i <= 2; i++) {
            row[i] = phaseStrings[transitionTable.currentState][i];
        }
    }

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

// INT0 ISR used to handle interrupts from MCP23S17.
ISR(INT0_vect) {
    Flag_Set(&flag_CheckExternalButtonInterrupts);
}

// PCINT1 used for S6 interrupts
ISR(PCINT1_vect) { Flag_Set(&flag_CheckInternalButtonInterrupts); }

// PCINT1 used for S3, s5 and hazard interrupts
ISR(PCINT2_vect) { 
    Flag_Set(&flag_CheckInternalButtonInterrupts); 
}

// Main Counter - 1 ms resolution
ISR(TIMER2_COMPA_vect) {
    // Add 1 millisecond to the system counter. Matter of priority so do that straight away.
    totalMillisecondsElasped++;
    ++timer2InterruptCount;
    Flag_Set(&flag_UpdateTimers);
}