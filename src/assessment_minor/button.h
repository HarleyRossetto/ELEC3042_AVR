#ifndef BUTTON_H
#define BUTTON_H

#include "stdint.h"
#include "bool.h"
#include "systemtimer.h"

typedef enum
{
    NONE, PRESSED, RELEASED, HELD
} ButtonState;

#define BUTTON_CHANGE_DELAY_MS 30

typedef struct {
    ButtonState currentState;
    uint64_t lastActionTime;
    uint8_t buttonPin;
    void(*eventPress)();
    void(*eventRelease)();
    bool updated;
    volatile uint8_t *port;
} Button;

// #define getButtonState(buttonPin, buttonInput) (*buttonInput & (1 << buttonPin) ? RELEASED : PRESSED)
// #define isPressed(buttonPin, buttonInput) !(*buttonInput & (1 << buttonPin))

void setTiming(volatile uint16_t *countReg, uint16_t ticks);
bool isPressed(volatile Button *btn);
void buttonPress(volatile Button *btn);
void buttonRelease(volatile Button *btn);
bool buttonTimingCorrect(volatile Button *btn);
void updateButton(volatile Button *btn);

#endif //BUTTON_H