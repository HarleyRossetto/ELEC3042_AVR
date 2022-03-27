#ifndef BUTTON_H
#define BUTTON_H

#include "avr/io.h"
#include "bool.h"
#include "stdint.h"
#include "systemtimer.h"
#include "tickable.h"

#define MAX_BUTTONS 3
typedef enum { NONE, PRESSED, RELEASED, HELD } ButtonState;

#define BUTTON_CHANGE_DELAY_MS 30

typedef struct {
    ButtonState currentState;
    uint64_t lastActionTime;
    uint8_t buttonPin;
    void (*eventPress)();
    void (*eventRelease)();
    bool updated;
    volatile uint8_t *port;
    Tickable *holdTimer;
} Button;

Button ButtonCreate(volatile uint8_t *ddr, volatile uint8_t *port, uint8_t pin, void (*pressEvent)(),
                    void (*releaseEvent)(), bool attachInterrupt);
void ButtonSetTiming(volatile uint16_t *countReg, uint16_t ticks);
bool ButtonIsPressed(volatile Button *btn);
bool ButtonIsReleased(volatile Button *btn);
bool ButtonIsTimingCorrect(volatile Button *btn);
void ButtonUpdate(volatile Button *btn);

#endif // BUTTON_H