#ifndef BUTTON_H
#define BUTTON_H

typedef enum
{
    NONE, PRESSED, RELEASED, HELD
} ButtonState;

#define BUTTON_CHANGE_DELAY_MS 30

typedef struct {
    ButtonState currentState;
    ButtonState actionState;
    uint64_t lastActionTime;
    uint8_t buttonPin;
    void(*eventPress)();
    void(*eventRelease)();
    bool updated;
} Button;

#endif //BUTTON_H