#include "button.h"

static volatile uint16_t *countRegister = 0;
static uint16_t ticksBetweenUpdates     = 0;

static Button buttons[MAX_BUTTONS];
static uint8_t buttonUsage = 0;

static void enterHeldState(void *button) {
    if (!button)
        return;

    Button *btn = (Button *)button;

    btn->currentState = HELD;
    btn->eventHeld();
}

Button *ButtonCreate(Port ddr, Port port, Port inputReg, uint8_t pin, void (*pressEvent)(),
                    void (*releaseEvent)(), bool attachInterrupt, Tickable *tickable) {
    *ddr &= ~(1 << pin); // Set port pin as input.
    *port |= (1 << pin); // Enabled pull-up resistor.

    if (buttonUsage > MAX_BUTTONS)
        return 0;

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
    buttons[buttonUsage] = (Button){inputReg, pin, RELEASED, 0, pressEvent, releaseEvent, tickable};
    tickable->eventCallback = enterHeldState;
    tickable->arg           = &buttons[buttonUsage];
    return &buttons[buttonUsage++];
}

inline Button *GetButtons() { return &buttons; }

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
                TickableDisable(btn->holdEvent);
                TickableReset(btn->holdEvent);
            }
            break;
        default:
            btn->currentState = RELEASED;
            break;
    }\
}

void ButtonUpdateAll() {
    for (int i = 0; i < buttonUsage; i++) {
        ButtonUpdate(&buttons[i]);
    }
}

void ButtonPress(volatile Button *btn) {
    btn->currentState = PRESSED;
}

void ButtonRelease(volatile Button *btn) {
    btn->currentState = RELEASED;
}

bool ButtonIsPressed(volatile Button *btn) { return !ButtonIsReleased(btn); }
bool ButtonIsReleased(volatile Button *btn) { return (*btn->inRegister & (1 << btn->buttonPin)); }