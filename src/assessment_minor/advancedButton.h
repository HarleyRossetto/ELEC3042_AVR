#ifndef ADVANCEDBUTTON_H
#define ADVANCEDBUTTON_H

#include "stdint.h"
#include "tickable.h"
#include "systemtimer.h"

#define BUTTON_CHANGE_DELAY_MS 30

typedef enum { ADV_PRESSED, ADV_RELEASED, ADV_HELD } AdvancedButtonState;

typedef void (*event)();

typedef volatile uint8_t *Port;

typedef uint8_t Pin;

typedef struct {
    AdvancedButtonState state;
    uint64_t actionTime;
    event pressEvent;
    event holdEvent;
    event releaseEvent;
    Tickable impluseEvent;
    Port port;
    Pin pin;
} AdvancedButton;

AdvancedButton *AdvBtnCreate(Port port, Pin pin, event press, event hold, event release);

void AdvBtnUpdateAll();
void AdvBtnUpdate(AdvancedButton *btn);

bool AdvBtnIsPressed(AdvancedButton *btn);
bool AdvBtnIsReleased(AdvancedButton *btn);

bool AdvBtnIsTimingCorrect(AdvancedButton *btn);

#endif // ADVANCEDBUTTON_H