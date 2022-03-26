#include "button.h"

static volatile uint16_t *countRegister = 0;
static uint16_t ticksBetweenUpdates       = 0;

// static Button buttons[MAX_BUTTONS];
// static uint8_t buttonUsage = 0;

Button ButtonCreate(volatile uint8_t *ddr, volatile uint8_t *port, uint8_t pin, 
                    void (*pressEvent)(), void (*releaseEvent)(), 
                    bool attachInterrupt) {
    *ddr &= ~(1 << pin);
    *port |= (1 << pin);

    if (attachInterrupt) {
        if (port == &PORTB) {
            PCMSK0 |= (1 << pin);  // Port B
            PCICR |= (1 << PCIE0);
        } else if (port == &PORTC) { // Port C
            PCMSK1 |= (1 << pin);
            PCICR |= (1 << PCIE1);
        } else if (port == &PORTD) { // Port D
            PCMSK2 |= (1 << pin);
            PCICR |= (1 << PCIE2);
        }
    }

    return (Button){RELEASED, 0, pin, pressEvent, releaseEvent, false, port};
}

void ButtonSetTiming(volatile uint16_t *countReg, uint16_t ticks) {
    countRegister     = countReg;
    ticksBetweenUpdates = ticks;
}

bool ButtonIsTimingCorrect(volatile Button *btn) {
    if (!btn)
        return false;

    return millisecondsElasped() - btn->lastActionTime > BUTTON_CHANGE_DELAY_MS;
}

void ButtonUpdate(volatile Button *btn) {
    if (!btn)
        return;

    // If the button was last considered released and is now pressed.
    if (btn->currentState == RELEASED && ButtonIsPressed(btn) && ButtonIsTimingCorrect(btn)) {
        // Calculate milliseconds elasped since last clock call.
        uint16_t millisElaspedAtInterruptCall = (*countRegister * 1000 / ticksBetweenUpdates);
        addMillisToSystemCounter(millisElaspedAtInterruptCall);

        ButtonPress(btn);
        // btn->currentState = PRESSED;
        if (btn->eventPress)
            btn->eventPress();
    }
    // Otherwise if button was last considered pressed and is now released.
    else if (btn->currentState == PRESSED && !ButtonIsPressed(btn) && ButtonIsTimingCorrect(btn)) {
        // Calculate milliseconds elasped since last clock call.
        uint16_t millisElaspedAtInterruptCall = (*countRegister * 1000 / ticksBetweenUpdates);
        addMillisToSystemCounter(millisElaspedAtInterruptCall);

        ButtonRelease(btn);
        // btn->currentState = RELEASED;
        if (btn->eventRelease)
            btn->eventRelease();
    }
}

void ButtonPress(volatile Button *btn) {
    btn->currentState = PRESSED;
    btn->updated      = true;
}

void ButtonRelease(volatile Button *btn) {
    btn->currentState = RELEASED;
    btn->updated      = true;
}

bool ButtonIsPressed(volatile Button *btn) { return !(*btn->port & (1 << btn->buttonPin)); }