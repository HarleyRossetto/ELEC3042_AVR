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

/// PROTOTYPES

typedef enum { BROADWAY_STRAIGHT, BROADWAY_TURN, LITTLE_STREET, PEDESTRIANS, HAZARD } Phase;
const char phaseStrings[][3] = {"BRS", "BRT", "LIT", "PED", "HAZ"};

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
Action actionUpdateDisplay(); // Debug
Action actionCheckButtonInterrupts();

void enableTimers();
void updateTimers();

const uint8_t LCD_Addr = 0x27;

void ttPeriodElasped();

void pwrSave();

uint32_t millisecondsElasped = 0;

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
    flag_CheckButtonInterrupts = Flag_Create(&actionCheckButtonInterrupts, NULL);
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
    tt_hazardCycle   = TimerTaskCreate(500L, &hazardState, NULL, true, false);
    tt_updateDisplay = TimerTaskCreate(100L, &Flag_Set, &flag_UpdateDisplay, true, false);
    tt_hazardCancel  = TimerTaskCreate(10000L, &endHazardState, NULL, false, true);
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

        Flag_RunIfSet(&flag_UpdateDisplay);

        Flag_RunIfSet(&flag_CheckButtonInterrupts);

        // Enter Power-Save Mode | OVERHEAD is very high to wakeup
        // pwrSave();
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

typedef struct {
    uint32_t lastTime;
    bool state;
} Sensor;
#define PRESSED 1
#define RELEASED 0

char sensorStateChar(Sensor *b) { return b->state ? 'X' : 'O'; }

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

Action actionUpdateDisplay() {
    uint8_t rega = MCP23S17_InterruptFlagReadRegister(MCP_PORT_A);

    LCD_Position(LCD_Addr, 0);
    LCD_Write(LCD_Addr, "INTFA:", 6);

    for (int i = 7; i >= 0; i--) {
        char c = ((rega >> i) & 1) + 48;
        LCD_Write(LCD_Addr, &c, 1);
    }

    uint8_t regb = MCP23S17_InterruptFlagReadRegister(MCP_PORT_B);

    LCD_Position(LCD_Addr, 0x40);
    LCD_Write(LCD_Addr, "INTFB:", 6);

    for (int i = 7; i >= 0; i--) {
        char c = ((regb >> i) & 1) + 48;
        LCD_Write(LCD_Addr, &c, 1);
    }
}

// Determines if sensor is pressed or not.
inline void
sensor(uint8_t flags, uint8_t changeBits, uint8_t pin, Sensor *sensor) {
    if (millisecondsElasped - sensor->lastTime < delay) {
        return;
    }

    sensor->lastTime = millisecondsElasped;
    if (flags & (1 << pin)) {
        if (changeBits & (1 << pin)) // Logic level high if left unpressed.
            sensor->state = RELEASED;
        else
            sensor->state = PRESSED;
    }
}

Action actionCheckButtonInterrupts() {

    uint8_t portAFlags = MCP23S17_InterruptFlagReadRegister(MCP_PORT_A);

    if (portAFlags) {
        uint8_t portAChangeBits = MCP23S17_InterruptCaptureReadRegister(MCP_PORT_A);
         // Sensor 1
        sensor(portAFlags, portAChangeBits, ICP4, &sensor1);

        // Sensor 0
        sensor(portAFlags, portAChangeBits, ICP0, &sensor0);
    }

    uint8_t portBFlags = MCP23S17_InterruptFlagReadRegister(MCP_PORT_B);

    if (portBFlags) {
        uint8_t portBChangeBits = MCP23S17_InterruptCaptureReadRegister(MCP_PORT_B);
        // Sensor 2
        sensor(portBFlags, portBChangeBits, ICP0, &sensor2);

        // Sensor 4
        sensor(portBFlags, portBChangeBits, ICP4, &sensor4);
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

// Main Counter
ISR(TIMER2_COMPA_vect) {
    // Add 1 millisecond to the system counter. Matter of priority so do that straight away.
    millisecondsElasped++;
    ++timer2InterruptCount;
    Flag_Set(&flag_UpdateTimers);
}