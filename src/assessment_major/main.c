#include "avr/interrupt.h"
#include "avr/io.h"
#include "bool.h"
#include "drivers/i2c/display/22S1_ELEC3042_I2C_PCF8574.h"
#include "drivers/mcp23s17/mcp23s17.h"
#include "drivers/spi/spi.h"
#include "flag.h"
#include "led.h"
#include "null.h"
#include "stdint.h"
#include "timerTask.h"
#include "timers.h"
#include "types.h"

/// PROTOTYPES

typedef enum { BROADWAY_STRAIGHT, BROADWAY_TURN, LITTLE_STREET, PEDESTRIANS, HAZARD } Phase;
const char phaseStrings[][3] = {"BRS", "BRT", "LIT", "PED", "HAZ"};

Phase phase;

typedef enum { Internal, External } LightLocation;

typedef enum { RED, YELLOW, GREEN} TrafficLightState;

typedef struct {
    uint8_t port; // Subtract 0x12 to obtain IODIR port address when in BANK mode. Otherwise subtract 0x09.
    uint8_t pin;
} ExternalLight;

typedef struct {
    Port port;
    uint8_t pin;
} InternalLight;

typedef struct {
    LightLocation interfaceLocation;
    ExternalLight externalInterface;
    InternalLight internalInterface;
} Light;

typedef struct {
    Light red;
    Light yellow;
    Light green;
    TrafficLightState activeLight;
} TrafficLight;

void trafficLight_Green(TrafficLight *light);
void trafficLight_Yellow(TrafficLight *light);
void trafficLight_Red(TrafficLight *light);
void trafficLight_Hazard(TrafficLight *light);

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
TimerTask **tt_hazardCycle;
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

Action actionUpdateStatusDisplay();
Action actionUpdateDisplay(); // Debug
Action actionCheckButtonInterrupts();

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

LED debugLed;

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

    // Enable interrupts on INT0
    EIMSK = (1 << INT0);
    // Respond to any logic level change on INT0
    EICRA = (0 << ISC01) | (1 << ISC00);

    // Flags
    initialiseFlags();

    Flag_Set(&flag_UpdateDisplay);

    // LED
    debugLed = LED_Create(&DDRB, &PORTB, PB0);

    initialiseMCP23S17();

    phase = HAZARD;

    initialiseTrafficLights();
    initialiseTimerTasks();
}

Initialiser initialiseMCP23S17() {
    // Enable Chip Select as output
    DDRB |= (1 << DDB2);
    PORTB |= (1 << PB2); // Raise CS

    // Initialise SPI and MCP23S17 consumer.
    initialiseSPIAsMaster(&DDRB, DDB3, DDB5);
    MCP23S17_Initialise(); // Must be initialised AFTER SPI.

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
        (Light){Internal, {NULL}, (InternalLight){&PORTD, PD6}},
        (Light){Internal, {NULL}, (InternalLight){&PORTD, PD6}}, // Duplicate red light to yellow so it becomes our hazard light too.
        (Light){Internal, {NULL}, (InternalLight){&PORTD, PD5}},
    };
}

void cycleAllLights() { 
    cycleLight(&tl_Broadway_North); 
    cycleLight(&tl_Broadway_South); 
    cycleLight(&tl_Broadway_South_Turn); 
    cycleLight(&tl_Little_Street); 
    cycleLight(&tl_Broadway_Pedestrian); 
}

void hazardState() {
    trafficLight_Hazard(&tl_Broadway_North); 
    trafficLight_Hazard(&tl_Broadway_South); 
    trafficLight_Hazard(&tl_Broadway_South_Turn); 
    trafficLight_Hazard(&tl_Little_Street); 
    trafficLight_Hazard(&tl_Broadway_Pedestrian); 
}

Initialiser initialiseTimerTasks() {
    //tt_lightCycle = TimerTaskCreate(1000L, &cycleAllLights, NULL, true, false);
    tt_period     = TimerTaskCreate(200L, &ttPeriodElasped, NULL, true, false);
    tt_hazardCycle   = TimerTaskCreate(1000L, &hazardState, NULL, true, false);
    tt_updateDisplay = TimerTaskCreate(500L, &Flag_Set, &flag_UpdateDisplay, true, false);
}

void enableTimers() {
    // Primary millisecond counter
    enableTimer(TC2, TIMER2_CLOCK_SELECT_64_PRESCALER);
}

void updateTimers() {
    TimerTaskUpdate(1 * timer2InterruptCount);
    timer2InterruptCount = 0;
}

void ttPeriodElasped() { periodCounter++; }

int main(void) {
    initialise();

    flag_UpdateTimers = Flag_Create(&updateTimers, NULL);

    // Enable timers now that we have done the rest of our configuration.
    enableTimers();

    // Enable interrupts.
    sei();

    while (1) {
        // Update timed tasks.
        Flag_RunIfSet(&flag_UpdateTimers);

        Flag_RunIfSet(&flag_UpdateDisplay);
        // actionUpdateDisplay();
        //  actionUpdateStatusDisplay();

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
Action actionUpdateStatusDisplay() {
    // Row row initially all spaces.
    char topRow[16] = {0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20};
    topRow[0]       = 'O';
    topRow[1]       = 'O';
    topRow[2]       = 'X';
    topRow[3]       = 'O';
    topRow[4]       = 'O';
    topRow[5]       = 'O';

    uint16_t period = periodCounter;
    for (int i = 4; i >= 0; i--) {
        topRow[11 + i] = (period % 10) + 48;
        period /= 10;
    }

    LCD_Position(LCD_Addr, 0x0);     // Top left first character position
    LCD_Write(LCD_Addr, topRow, 16); // Write Button States.

    LCD_Position(LCD_Addr, 0x40);                 // Bottom left first character position
    LCD_Write(LCD_Addr, &phaseStrings[phase], 3); // Write System Phase.
    LCD_Write(LCD_Addr, "R", 1);                  // Write phase color
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

Action actionCheckButtonInterrupts() {
    MCP23S17_ToggleBit(GPIOA, GP7);
    LCD_Position(LCD_Addr, 0x40);
    LCD_Write(LCD_Addr, "x", 1);
}

void clearLight_Internal(InternalLight *light) {
    if (!light)
        return;

    *light->port &= ~(1 << light->pin);
}

void setLight_Internal(InternalLight *light) {
    if (!light)
        return;

    *light->port |= (1 << light->pin);
}

void toggleLight_Internal(InternalLight *light) {
    if (!light)
        return;

    *light->port ^= (1 << light->pin);
}

void clearLight_External(ExternalLight *light) {
    if (!light)
        return;

    MCP23S17_GpioClearPin(light->port, light->pin);
}

void setLight_External(ExternalLight *light) {
    if (!light)
        return;

    MCP23S17_GpioSetPin(light->port, light->pin);
}

void toggleLight_External(ExternalLight *light) {
    if (!light)
        return;

    MCP23S17_GpioTogglePin(light->port, light->pin);
}

void clearLight(Light *light) {
    if (!light)
        return;

    if (light->interfaceLocation == Internal) {
        clearLight_Internal(&light->internalInterface);
    } else if (light->interfaceLocation == External) {
        clearLight_External(&light->externalInterface);
    }
}

void setLight(Light *light) {
    if (!light)
        return;

    if (light->interfaceLocation == Internal) {
        setLight_Internal(&light->internalInterface);
    } else if (light->interfaceLocation == External) {
        setLight_External(&light->externalInterface);
    }
}

void toggleLight(Light *light) {
    if (!light)
        return;

    if (light->interfaceLocation == Internal) {
        toggleLight_Internal(&light->internalInterface);
    } else if (light->interfaceLocation == External) {
        toggleLight_External(&light->externalInterface);
    }
}

void trafficLight_Green(TrafficLight *light) {
    if (!light)
        return;

    clearLight(&light->red);
    clearLight(&light->yellow);
    setLight(&light->green);
}

void trafficLight_Yellow(TrafficLight *light) {
    clearLight(&light->red);
    setLight(&light->yellow);
    clearLight(&light->green);
}

void trafficLight_Red(TrafficLight *light) {
    setLight(&light->red);
    clearLight(&light->yellow);
    clearLight(&light->green);
}

void trafficLight_Hazard(TrafficLight *light) {
    if (!light)
        return;

    toggleLight(&light->yellow);
}

// INT0 ISR used to handle interrupts from MCP23S17.
ISR(INT0_vect) {
    // cli();
    // Flag_Set(&flag_CheckButtonInterrupts);
    // uint8_t portAFlags = MCP23S17_InterruptFlagReadRegister(MCP_PORT_A);
    // uint8_t portBFlags = 0;

    // uint8_t portAChangeBit, portBChangeBit;

    // If nothing found on port A, then read port B.
    // if (!portAFlags) {
    //     portBFlags = MCP23S17_InterruptFlagReadRegister(MCP_PORT_B);
    //     //portBChangeBit =
    // }

    MCP23S17_InterruptCaptureReadRegister(MCP_PORT_A);

    MCP23S17_InterruptCaptureReadRegister(MCP_PORT_B);

    // LED_Toggle(&debugLed);
    //  sei();
}

void cycleLight(TrafficLight *light) {
    if (!light)
        return;

    switch (light->activeLight) {
        case RED:
            trafficLight_Green(light);
            light->activeLight = GREEN;
            break;
        case YELLOW:
            trafficLight_Red(light);
            light->activeLight = RED;
            break;
        case GREEN:
            trafficLight_Yellow(light);
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
    ++timer2InterruptCount;
    Flag_Set(&flag_UpdateTimers);
}