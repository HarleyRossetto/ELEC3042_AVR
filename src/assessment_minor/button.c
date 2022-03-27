#include "button.h"

static volatile uint16_t *countRegister = 0;
static uint16_t ticksBetweenUpdates     = 0;

// static Button buttons[MAX_BUTTONS];
// static uint8_t buttonUsage = 0;

Button ButtonCreate(Port ddr, Port port, Port inputReg, uint8_t pin, void (*pressEvent)(),
                    void (*releaseEvent)(), bool attachInterrupt, Tickable *tickable) {
    *ddr &= ~(1 << pin); // Set port pin as input.
    *port |= (1 << pin); // Enabled pull-up resistor.

    if (attachInterrupt) {
        if (port == &PORTB) {        // Port B
            PCMSK0 |= (1 << pin); 
            PCICR |= (1 << PCIE0);
        } else if (port == &PORTC) { // Port C
            PCMSK1 |= (1 << pin);
            PCICR |= (1 << PCIE1);
        } else if (port == &PORTD) { // Port D
            PCMSK2 |= (1 << pin);
            PCICR |= (1 << PCIE2);
        }
    }
    return (Button){inputReg, pin, RELEASED, 0, pressEvent, releaseEvent, tickable};
}

void ButtonSetTiming(volatile uint16_t *countReg, uint16_t ticks) {
    countRegister       = countReg;
    ticksBetweenUpdates = ticks;
}

bool ButtonIsTimingCorrect(volatile Button *btn) {
    if (!btn)
        return false;

    uint64_t delta = millisecondsElasped() - btn->lastActionTime;

    return delta > BUTTON_CHANGE_DELAY_MS;
}

void ButtonUpdate(volatile Button *btn) {
    //Return if pointer is null or the timing is not correct (debounce duration not exceeded yet).
    if (!btn || !ButtonIsTimingCorrect(btn))
        return;
    
    switch (btn->currentState) {
        case RELEASED:
            // If currently released and the button is pressed start the hold timer.
            if (ButtonIsPressed(btn)) {
                btn->currentState = PRESSED;
                btn->lastActionTime = millisecondsElasped();
                // Start the hold event timer
                if (btn->holdEvent)
                    TickableEnable(btn->holdEvent);
            }
            break;
        case PRESSED:
            //If button is released
            if (ButtonIsReleased(btn)) {
                btn->currentState = RELEASED;
                btn->lastActionTime = millisecondsElasped();
                if (btn->holdEvent)
                    TickableDisable(btn->holdEvent);
                TickableReset(btn->holdEvent);
                btn->eventPress();
            }
            break;
        case HELD:
            if (ButtonIsReleased(btn)) {
                btn->currentState = RELEASED;
                btn->lastActionTime = millisecondsElasped();
                btn->eventRelease();
            }
            break;
        default:
            btn->currentState = RELEASED;
            break;
    }

    // // If the button was last considered released and is now pressed.
    // if (btn->currentState == RELEASED && ButtonIsPressed(btn) && ButtonIsTimingCorrect(btn)) {
    //     btn->lastActionTime = millisecondsElasped();

    //     ButtonPress(btn);
    //     // btn->currentState = PRESSED;
    //     if (btn->eventPress)
    //         btn->eventPress();
    // }
    // // Otherwise if button was last considered pressed and is now released.
    // else if (btn->currentState == PRESSED && !ButtonIsPressed(btn) && ButtonIsTimingCorrect(btn)) {
    //     // Calculate milliseconds elasped since last clock call.
    //     btn->lastActionTime = millisecondsElasped();

    //     ButtonRelease(btn);
    //     // btn->currentState = RELEASED;
    //     if (btn->eventRelease)
    //         btn->eventRelease();
    //}
}

void ButtonPress(volatile Button *btn) {
    btn->currentState = PRESSED;
}

void ButtonRelease(volatile Button *btn) {
    btn->currentState = RELEASED;
}

bool ButtonIsPressed(volatile Button *btn) { return !ButtonIsReleased(btn); }
bool ButtonIsReleased(volatile Button *btn) { return (*btn->inRegister & (1 << btn->buttonPin)); }