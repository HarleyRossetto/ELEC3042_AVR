#include "advancedButton.h"

#define MAX_ADVANCED_BUTTONS 3

static AdvancedButton buttons[MAX_ADVANCED_BUTTONS];
static uint8_t buttonUsage = 0;

static void impluseEvent(void *arg) {
    if (!arg)
        return;

    AdvancedButton *btn = (AdvancedButton *)arg;

    if (AdvBtnIsPressed(btn)) {
        btn->state = ADV_HELD;
        AdvBtnUpdate(btn);
    } else {
        btn->pressEvent();
    }
}

AdvancedButton *AdvBtnCreate(Port port, Pin pin, event press, event hold, event release) {
    Tickable t                        = *TickableCreate(30, impluseEvent, (void *)&buttons[buttonUsage], false, true);
    buttons[buttonUsage]              = (AdvancedButton){ADV_RELEASED, 0L, press, hold, release, t, port, pin};
    // buttons[buttonUsage].impluseEvent = t;
    return &buttons[buttonUsage++];
}

void AdvBtnUpdateAll() {
    for (int i = 0; i < buttonUsage; i++) {
        AdvBtnUpdate(&buttons[i]);
    }
}

void AdvBtnUpdate(AdvancedButton *btn) {
    // Ensure pointer is valid, and debounce button.
    if (!btn || !AdvBtnIsTimingCorrect(btn))
        return;

    switch (btn->state) {
        case ADV_PRESSED:
            if (AdvBtnIsReleased(btn)) {
                btn->state = ADV_RELEASED;
            }
            break;
        case ADV_HELD:
            if (AdvBtnIsReleased(btn)) {
                btn->state      = ADV_RELEASED;
                btn->actionTime = millisecondsElasped();
            }
            break;
        case ADV_RELEASED:
            // Button considered pressed.
            if (AdvBtnIsPressed(btn)) {
                btn->actionTime = millisecondsElasped();
                btn->state      = ADV_PRESSED;
                TickableEnable(&btn->impluseEvent);
            }
            break;
        default:
            btn->state = ADV_RELEASED;
            break;
    }
}

inline bool AdvBtnIsPressed(AdvancedButton *btn) { return !(*btn->port & (1 << btn->pin)); }
inline bool AdvBtnIsReleased(AdvancedButton *btn) { return !AdvBtnIsPressed(btn); }

bool AdvBtnIsTimingCorrect(AdvancedButton *btn) {
    if (!btn)
        return false;

    return millisecondsElasped() - btn->actionTime > BUTTON_CHANGE_DELAY_MS;
}