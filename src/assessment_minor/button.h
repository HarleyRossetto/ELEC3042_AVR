#ifndef BUTTON_H
#define BUTTON_H

#include "stdint.h"
#include "bool.h"
#include "systemtimer.h"
#include "avr/io.h"
#include "tickable.h"

#define MAX_BUTTONS 3
typedef enum
{
    RELEASED, PRESSED, HELD
} ButtonState;

#define BUTTON_CHANGE_DELAY_MS 15

typedef volatile uint8_t *Port;
typedef Port InputRegister;
typedef uint8_t Pin;

typedef struct {
    Port inRegister;
    Pin buttonPin;
    ButtonState currentState;
    uint64_t lastActionTime;
    void(*eventPress)();
    void(*eventHeld)();
    Tickable *holdEvent;
} Button;

Button *ButtonCreate(Port ddr, Port port, Port, Pin  pin, void (*pressEvent)(), void (*releaseEvent)(), bool attachInterrupt, 
                    Tickable *tickable);
void ButtonSetTiming(volatile uint16_t *countReg, uint16_t ticks);
bool ButtonIsPressed(volatile Button *btn);
bool ButtonIsReleased(volatile Button *btn);
void ButtonPress(volatile Button *btn);
void ButtonRelease(volatile Button *btn);
bool ButtonIsTimingCorrect(volatile Button *btn);
void ButtonUpdate(volatile Button *btn);
void ButtonUpdateAll();
Button *GetButtons();

#endif //BUTTON_H