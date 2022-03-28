#ifndef BUTTON_H
#define BUTTON_H

#include "stdint.h"
#include "bool.h"
#include "systemtimer.h"
#include "avr/io.h"
#include "timerTask.h"

typedef enum
{
    RELEASED, PRESSED, HELD
} ButtonState;

#define BUTTON_CHANGE_DELAY_MS 15

typedef volatile uint8_t *Port;
typedef Port InputRegister;
typedef uint8_t Pin;

typedef enum { FLAG_CLEAR, FLAG_PRESSED, FLAG_HELD, FLAG_UNKNOWN } ButtonActionFlag;

typedef struct {
    Port inRegister;
    Pin buttonPin;
    ButtonState currentState;
    uint64_t lastActionTime;
    void(*eventPress)();
    void(*eventHeld)();
    TimerTask *holdTimerTask;
    ButtonActionFlag actionFlag;
} Button;

#define MAX_BUTTONS 3
typedef Button ButtonArray[MAX_BUTTONS];

Button *ButtonCreate(Port ddr, Port port, Port, Pin  pin, void (*pressEvent)(), void (*releaseEvent)(), bool attachInterrupt, 
                    TimerTask *timerTask);
void ButtonSetTiming(uint16_t *countReg, uint16_t ticks);
bool ButtonIsPressed(Button *btn);
bool ButtonIsReleased(Button *btn);
void ButtonPress(Button *btn);
void ButtonRelease(Button *btn);
bool ButtonIsTimingCorrect(Button *btn);
void ButtonUpdate(Button *btn);
void ButtonUpdateAll();
void GetButtons(ButtonArray *btns);
void ButtonClearAllFlags();
void ButtonSetPressFlag(Button *btn);
void ButtonSetHoldFlag(Button *btn);
ButtonActionFlag ButtonReadFlag(Button *btn);

#endif //BUTTON_H