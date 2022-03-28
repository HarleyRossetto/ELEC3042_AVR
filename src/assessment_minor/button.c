#include "button.h"
#include "avr/io.h"

static uint16_t *countRegister      = 0;
static uint16_t ticksBetweenUpdates = 0;

static ButtonArray buttons;
static uint8_t buttonUsage = 0;

static void enterHeldState(void *button) {
    if (!button)
        return;

    Button *btn = (Button *)button;

    btn->currentState = HELD;
    ButtonSetHoldFlag(btn);
    if (btn->eventHeld)
        btn->eventHeld();
}

Button *ButtonCreate(Port ddr, Port port, Port inputReg, uint8_t pin, void (*pressEvent)(), void (*holdEvent)(), bool attachInterrupt,
                     TimerTask *timerTask) {
    *ddr &= ~(1 << pin); // Set port pin as input.
    *port |= (1 << pin); // Enabled pull-up resistor.

    if (buttonUsage > MAX_BUTTONS)
        return 0;

    if (attachInterrupt) {
        if (port == &PORTB) { // Port B
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
    buttons[buttonUsage]     = (Button){inputReg, pin, RELEASED, 0, pressEvent, holdEvent, timerTask, FLAG_CLEAR};
    timerTask->eventCallback = enterHeldState;
    timerTask->arg           = &buttons[buttonUsage];
    return &buttons[buttonUsage++];
}

void GetButtons(ButtonArray *btns) { btns = &buttons; }
// void GetButtons(Button *(btnArray[MAX_BUTTONS])) { btnArray = &buttons; }

// inline Button[] *GetButtons() { return &buttons; }

void ButtonSetTiming(uint16_t *countReg, uint16_t ticks) {
    countRegister       = countReg;
    ticksBetweenUpdates = ticks;
}

bool ButtonIsTimingCorrect(Button *btn) {
    if (!btn)
        return false;

    uint64_t delta = millisecondsElasped() - btn->lastActionTime;

    return delta > BUTTON_CHANGE_DELAY_MS;
}

static inline void ButtonClearFlag(Button *btn) {
    if (btn)
        btn->actionFlag = FLAG_CLEAR;
}

void ButtonUpdate(Button *btn) {
    // Return if pointer is null or the timing is not correct (debounce duration not exceeded yet).
    if (!btn || !ButtonIsTimingCorrect(btn))
        return;

    switch (btn->currentState) {
        case RELEASED:
            // If currently released and the button is pressed start the hold timer.
            if (ButtonIsPressed(btn)) {
                btn->currentState   = PRESSED;
                btn->lastActionTime = millisecondsElasped();
                // Start the hold event timer
                if (btn->holdTimerTask)
                    TimerTaskEnable(btn->holdTimerTask);
            }
            break;
        case PRESSED:
            // If button is released
            if (ButtonIsReleased(btn)) {
                btn->currentState   = RELEASED;
                btn->lastActionTime = millisecondsElasped();
                if (btn->holdTimerTask)
                    TimerTaskDisable(btn->holdTimerTask);
                TimerTaskReset(btn->holdTimerTask);
                ButtonSetPressFlag(btn);
                if (btn->eventPress)
                    btn->eventPress();
            }
            break;
        case HELD:
            if (ButtonIsReleased(btn)) {
                btn->currentState   = RELEASED;
                btn->lastActionTime = millisecondsElasped();
                ButtonClearFlag(btn);
                TimerTaskDisable(btn->holdTimerTask);
                TimerTaskReset(btn->holdTimerTask);
            }
            break;
        default:
            btn->currentState = RELEASED;
            break;
    }
}

void ButtonClearAllFlags() {
    for (int i = 0; i < buttonUsage; i++) {
        ButtonClearFlag(&buttons[i]);
    }
}

void ButtonSetPressFlag(Button *btn) { btn->actionFlag = FLAG_PRESSED; }
void ButtonSetHoldFlag(Button *btn) { btn->actionFlag = FLAG_HELD; }

ButtonActionFlag ButtonReadFlag(Button *btn) {
    if (!btn)
        return FLAG_UNKNOWN;
    return btn->actionFlag;
    // ButtonActionFlag f = btn->actionFlag;
    // ButtonClearFlag(btn);
    // return f;
}

void ButtonUpdateAll() {
    for (int i = 0; i < buttonUsage; i++) {
        ButtonUpdate(&buttons[i]);
    }
}

void ButtonPress(Button *btn) { btn->currentState = PRESSED; }

void ButtonRelease(Button *btn) { btn->currentState = RELEASED; }

bool ButtonIsPressed(Button *btn) { return !ButtonIsReleased(btn); }
bool ButtonIsReleased(Button *btn) { return (*btn->inRegister & (1 << btn->buttonPin)); }