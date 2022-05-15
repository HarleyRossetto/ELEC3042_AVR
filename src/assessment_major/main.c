#include "avr/interrupt.h"
#include "avr/io.h"
#include "bool.h"
#include "drivers/i2c/display/22S1_ELEC3042_I2C_PCF8574.h"
#include "drivers/mcp23s17/mcp23s17.h"
#include "flag.h"
#include "intersection/light.h"
#include "intersection/trafficLight.h"
#include "null.h"
#include "stdint.h"
#include "timerTask.h"
#include "timers.h"
#include "types.h"
#include "sensor.h"
#include "systemtimer.h"
#include "adc.h"

// ADV value to Period Time MS Lookup table.
const uint16_t ADC_TO_PERIOD_TIME_MS_LUT[] = {80, 83, 87, 90, 94, 98, 101, 105, 108, 112, 116, 119, 123, 126, 130, 134, 137, 141, 144, 148, 152, 155, 159, 162, 166, 170, 173, 177, 181, 184, 188, 191, 195, 199, 202, 206, 209, 213, 217, 220, 224, 227, 231, 235, 238, 242, 245, 249, 253, 256, 260, 264, 267, 271, 274, 278, 282, 285, 289, 292, 296, 300, 303, 307, 310, 314, 318, 321, 325, 328, 332, 336, 339, 343, 346, 350, 354, 357, 361, 365, 368, 372, 375, 379, 383, 386, 390, 393, 397, 401, 404, 408, 411, 415, 419, 422, 426, 429, 433, 437, 440, 444, 448, 451, 455, 458, 462, 466, 469, 473, 476, 480, 484, 487, 491, 494, 498, 502, 505, 509, 512, 516, 520, 523, 527, 530, 534, 538, 541, 545, 549, 552, 556, 559, 563, 567, 570, 574, 577, 581, 585, 588, 592, 595, 599, 603, 606, 610, 613, 617, 621, 624, 628, 632, 635, 639, 642, 646, 650, 653, 657, 660, 664, 668, 671, 675, 678, 682, 686, 689, 693, 696, 700, 704, 707, 711, 714, 718, 722, 725, 729, 733, 736, 740, 743, 747, 751, 754, 758, 761, 765, 769, 772, 776, 779, 783, 787, 790, 794, 797, 801, 805, 808, 812, 816, 819, 823, 826, 830, 834, 837, 841, 844, 848, 852, 855, 859, 862, 866, 870, 873, 877, 880, 884, 888, 891, 895, 898, 902, 906, 909, 913, 917, 920, 924, 927, 931, 935, 938, 942, 945, 949, 953, 956, 960, 963, 967, 971, 974, 978, 981, 985, 989, 992, 996, 1000};
/**
 * 
 * Average ADC over 10 samples.
 * 
 */


/// PROTOTYPES
typedef enum { BROADWAY_STRAIGHT, BROADWAY_TURN, LITTLE_STREET, PEDESTRIANS, HAZARD } Phase;
const char phaseStrings[][3] = {"BRS", "BRT", "LIT", "PED", "HAZ"};

extern volatile uint64_t totalMillisecondsElasped;

Phase phase;

TrafficLight tl_Broadway_North;
TrafficLight tl_Broadway_South;
TrafficLight tl_Broadway_South_Turn;
TrafficLight tl_Broadway_Pedestrian;
TrafficLight tl_Little_Street;

TimerTask *tt_lightCycle;
volatile uint8_t lightCycleState;

void cycleLight(TrafficLight *light);

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
Flag flag_CheckButtonInterrupts;

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

    // Internal LED
    DDRD |= (1 << DDD6) | (1 << DDD5);

    // Hazard Interrupt Not really worth it is it?
    // PCMSK0 |= (1 << PC3); // D3/Pin 18
    // PCICR |= (1 << PCIE0);
    // Hazard Pull-up
    DDRD &= ~(1 << DDD3);
    PORTD |= (1 << PD3);

    phase = HAZARD;
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
    flag_UpdateDisplay         = Flag_Create(&actionUpdateStatusDisplay, NULL);
    flag_CheckButtonInterrupts = Flag_Create(&actionCheckMcp23s17ButtonInterrupts, NULL);
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
}

Initialiser initialiseInterrupt0() {
    // Enable interrupts on INT0
    EIMSK = (1 << INT0);
    // Respond to any logic level change on INT0
    EICRA = (0 << ISC01) | (1 << ISC00);
}

void cycleAllLights() {
    cycleLight(&tl_Broadway_North);
    cycleLight(&tl_Broadway_South);
    cycleLight(&tl_Broadway_South_Turn);
    cycleLight(&tl_Little_Street);
    cycleLight(&tl_Broadway_Pedestrian);
}

void hazardState() {
    TrafficLight_Hazard(&tl_Broadway_North);
    TrafficLight_Hazard(&tl_Broadway_South);
    TrafficLight_Hazard(&tl_Broadway_South_Turn);
    TrafficLight_Hazard(&tl_Little_Street);
    TrafficLight_Hazard(&tl_Broadway_Pedestrian);
}

void clearAllLights() {
    TrafficLight_Blank(&tl_Broadway_North);
    TrafficLight_Blank(&tl_Broadway_South);
    TrafficLight_Blank(&tl_Broadway_South_Turn);
    TrafficLight_Blank(&tl_Little_Street);
    TrafficLight_Blank(&tl_Broadway_Pedestrian);
}

void endHazardState() {
    TimerTaskDisable(tt_hazardCycle);
    clearAllLights();
    TrafficLight_Set(&tl_Broadway_Pedestrian.green);
}

Initialiser initialiseTimerTasks() {
    // tt_lightCycle = TimerTaskCreate(1000L, &cycleAllLights, NULL, true, false);

    tt_period        = TimerTaskCreate(1000L, &ttPeriodElasped, NULL, true, false);
    tt_hazardCycle   = TimerTaskCreate(1000L, &cycleAllLights, NULL, true, false); //&hazardState
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
}

typedef enum { SS_HAZARD, SS_NORMAL } SystemState;

SystemState ss = SS_HAZARD;

int main(void) {
    initialise();

    flag_UpdateTimers = Flag_Create(&updateTimers, NULL);

    // Enable timers now that we have done the rest of our configuration.
    enableTimers();

    // Enable interrupts.
    sei();

    while (1) {
        // Handles Hazard / Normal Operation States
        // If no hazard signal
        if (PIND & (1 << PIND3)) {
            if (!tt_hazardCancel->enabled && ss == SS_HAZARD) {
                periodCounter = 0;
                ss            = SS_NORMAL;
                TimerTaskReset(tt_hazardCancel);
                TimerTaskEnable(tt_hazardCancel);
            }
        } else { // Hazard signal
            ss = SS_HAZARD;
            TimerTaskEnable(tt_hazardCycle);
            TrafficLight_Clear(&tl_Broadway_Pedestrian.green);
        }

        // Update timed tasks.
        Flag_RunIfSet(&flag_UpdateTimers);

        // Update Display
        Flag_RunIfSet(&flag_UpdateDisplay);

        // Check MCP23S17 Interrupts
        Flag_RunIfSet(&flag_CheckButtonInterrupts);
    }

    return 0;
}

// Break these out into individual functions, then when a value is updated
// we can just set the cursor accordingly and update said character, not
// having to bother about the entire display, want to save as much time
// as possible by avoid i2c transfers.
//
// Batch all the data into rows to limit i2c calls potentially?
const uint8_t delay = 10;

char sensorStateChar(Sensor *b) { return b->triggered ? 'X' : 'O'; }

Sensor sensor0 = {0, RELEASED};
Sensor sensor1 = {0, RELEASED};
Sensor sensor2 = {0, RELEASED};
Sensor sensor3 = {0, RELEASED};
Sensor sensor4 = {0, RELEASED};
Sensor sensor5 = {0, RELEASED};
Sensor sensor6 = {0, RELEASED};

char portAFlagsDisplay[8] = {48, 48, 48, 48, 48, 48, 48, 48};

Action actionUpdateStatusDisplay() {
    // Row row initially all spaces.
    char topRow[16] = {0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20};
    topRow[0]       = sensorStateChar(&sensor0);
    topRow[1]       = sensorStateChar(&sensor1);
    topRow[2]       = sensorStateChar(&sensor2);
    topRow[3]       = sensorStateChar(&sensor3);
    topRow[4]       = sensorStateChar(&sensor4);
    topRow[5]       = sensorStateChar(&sensor5);
    // topRow[6]       = sensorStateChar(s6);

    uint16_t period = periodCounter;
    for (int i = 4; i >= 0; i--) {
        topRow[11 + i] = (period % 10) + 48;
        period /= 10;
    }

    LCD_Position(LCD_Addr, 0x0);     // Top left first character position
    LCD_Write(LCD_Addr, topRow, 16); // Write Button States.

    LCD_Position(LCD_Addr, 0x40);                 // Bottom left first character position
    LCD_Write(LCD_Addr, &phaseStrings[phase], 3); // Write System Phase.
    LCD_Write(LCD_Addr, "Y ", 2);                  // Write phase color
    LCD_Write(LCD_Addr, &portAFlagsDisplay, 8);
}

Action actionCheckMcp23s17ButtonInterrupts() {
    uint8_t portAFlags = MCP23S17_InterruptFlagReadRegister(MCP_PORT_A);

    if (portAFlags) {
        uint8_t portAChangeBits = MCP23S17_InterruptCaptureReadRegister(MCP_PORT_A);
         // Sensor 1
        Sensor_CheckState(portAFlags, portAChangeBits, ICP4, &sensor1);

        // Sensor 0
        Sensor_CheckState(portAFlags, portAChangeBits, ICP0, &sensor0);
    }

    uint8_t portBFlags = MCP23S17_InterruptFlagReadRegister(MCP_PORT_B);

    if (portBFlags) {
        uint8_t portBChangeBits = MCP23S17_InterruptCaptureReadRegister(MCP_PORT_B);
        // Sensor 2
        Sensor_CheckState(portBFlags, portBChangeBits, ICP0, &sensor2);

        // Sensor 4
        Sensor_CheckState(portBFlags, portBChangeBits, ICP4, &sensor4);
    }
}

// INT0 ISR used to handle interrupts from MCP23S17.
ISR(INT0_vect) {
    Flag_Set(&flag_CheckButtonInterrupts);
}

void cycleLight(TrafficLight *light) {
    if (!light)
        return;

    switch (light->activeLight) {
        case RED:
            TrafficLight_Green(light);
            light->activeLight = GREEN;
            break;
        case YELLOW:
            TrafficLight_Red(light);
            light->activeLight = RED;
            break;
        case GREEN:
            TrafficLight_Yellow(light);
            light->activeLight = YELLOW;
            break;
    }
}

// Puts the MCU into Power-save mode. (DS Page 50, 10.6)
// Active Clock domains the wake-up sources as follows:
// clk_asy, Timer Oscillator Enabled, INT{1|0} & Pin Change, TWI Address Match, Timer2, WDT, Software BOD Disable.
void pwrSave() {
    cli();
    SMCR |= (1 << SM1) | (1 << SM0) | (1 << SE); // Power-Save mode.
    sei();
    asm("SLEEP"); // Enter Sleep
    // SMCR &= ~(1 << SE); // Disable Sleep
}



// Main Counter - 1 ms resolution
ISR(TIMER2_COMPA_vect) {
    // Add 1 millisecond to the system counter. Matter of priority so do that straight away.
    totalMillisecondsElasped++;
    ++timer2InterruptCount;
    Flag_Set(&flag_UpdateTimers);
}