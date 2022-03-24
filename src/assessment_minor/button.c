#include "button.h"

static volatile uint16_t *countRegister = 0;
static uint16_t ticksPer100Millis = 0;

void setTiming(volatile uint16_t *countReg, uint16_t ticks) {
    countRegister = countReg;
    ticksPer100Millis = ticks;
}

bool buttonTimingCorrect(volatile Button *btn) {
    if (!btn)
        return false;

    return millisecondsElasped() - btn->lastActionTime > BUTTON_CHANGE_DELAY_MS;
}

void updateButton(volatile Button *btn) {
      if (!btn)
        return;

    // If the button was last considered released and is now pressed.
    if (btn->currentState == RELEASED && isPressed(btn) && buttonTimingCorrect(btn))
    {
         //Calculate milliseconds elasped since last clock call.
        uint16_t  millisElaspedAtInterruptCall = (*countRegister * 1000 / ticksPer100Millis);
        addMillisToSystemCounter(millisElaspedAtInterruptCall);

        buttonPress(btn);
        // btn->currentState = PRESSED;
        if (btn->eventPress)
            btn->eventPress();
    }
    // Otherwise if button was last considered pressed and is now released.
    else if (btn->currentState == PRESSED && !isPressed(btn) && buttonTimingCorrect(btn)) 
    {
         //Calculate milliseconds elasped since last clock call.
        uint16_t  millisElaspedAtInterruptCall = (*countRegister * 1000 / ticksPer100Millis);
        addMillisToSystemCounter(millisElaspedAtInterruptCall);

        buttonRelease(btn);
        // btn->currentState = RELEASED;
        if (btn->eventRelease)
            btn->eventRelease();
    }
}

void buttonPress(volatile Button* btn) {
    btn->currentState = PRESSED;
    btn->updated = true;
}

void buttonRelease(volatile Button* btn) {
    btn->currentState = RELEASED;
    btn->updated = true;
}

bool isPressed(volatile Button *btn) {
    return !(*btn->port & (1 << btn->buttonPin));
}