#ifndef BUTTON_H
#define BUTTON_H

#include "stdint.h"
#include "bool.h"
#include "systemtimer.h"
#include "avr/io.h"

#define MAX_BUTTONS 3
typedef enum
{
    NONE, PRESSED, RELEASED, HELD
} ButtonState;

#define BUTTON_CHANGE_DELAY_MS 15

typedef struct {
    ButtonState currentState;
    uint64_t lastActionTime;
    uint8_t buttonPin;
    void(*eventPress)();
    void(*eventRelease)();
    bool updated;
    volatile uint8_t *port;
} Button;

Button ButtonCreate(volatile uint8_t *ddr, volatile uint8_t *port, uint8_t pin, void (*pressEvent)(), void (*releaseEvent)(), bool attachInterrupt);
void ButtonSetTiming(volatile uint16_t *countReg, uint16_t ticks);
bool ButtonIsPressed(volatile Button *btn);
void ButtonPress(volatile Button *btn);
void ButtonRelease(volatile Button *btn);
bool ButtonIsTimingCorrect(volatile Button *btn);
void ButtonUpdate(volatile Button *btn);

#endif //BUTTON_H