/**
 * @file button.c
  * @author Harley Rossetto (44618883@students.mq.edu.au)
 * @brief Button abstraction types and functions.
 * Each button is stored as a struct which maintains each of their own states.
 * @version 0.1
 * @date 2022-04-06
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "button.h"
#include "avr/io.h"

#include "null.h"
#include "systemtimer.h"

// Prototypes for local functions.
static inline void ButtonActionFlagSetHold(Button *btn);
static inline void ButtonActionFlagSetPress(Button *btn);
static inline void ButtonActionFlagSetClear(Button *btn);
static bool ButtonIsTimingCorrect(Button *btn);
static void enterHeldState(void *button);

static ButtonArray buttons;
static uint8_t buttonUsage = 0;

/**
 * @brief Creates a new struct Button instance, storing it in the ButtonArray.
 * 
 * @param ddr Data Direction Register the button is on.
 * @param port Port Register the button is on.
 * @param pin Pin on the Port/DDR the button is on.
 * @param pressEvent The callback event to call when the button is pressed (Pressed then released in a sub 2 second period).
 * @param holdEvent The callback event to call when the button is held (Pressed and held for 2 seconds).
 * @param attachInterrupt Whether or not to respond to interrrupts on the pin.
 * @return Button* A reference to the newly created Button.
 */
Button *ButtonCreate(Port ddr, Port port, Port inputReg, uint8_t pin, void (*pressEvent)(), void (*holdEvent)(), bool attachInterrupt) {
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
    // Configure the hold timer for triggering the hold even after 2 seconds.
    TimerTask *holdTimer      = TimerTaskCreate(2000L, enterHeldState, &buttons[buttonUsage], false, true);
    buttons[buttonUsage] = (Button){inputReg, pin, RELEASED, 0, pressEvent, holdEvent, holdTimer, FLAG_CLEAR};

    return &buttons[buttonUsage++];
}

/**
 * @brief Provides a reference to the internal button array in the provided parameter.
 * 
 * @param btns A pointer to a ButtonArray in which to store a reference to the internal buttons array in.
 */
void ButtonsGetAll(ButtonArray *btns) { btns = &buttons; }

/**
 * @brief Updates a Buttons state (Pressed, Released, Held). 
 * If *btn null or debounce time insufficient function does nothing and returns.
 * @param btn Button to update. 
 */
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
                // Start the hold event timer if it exists.
                if (btn->holdTimerTask)
                    TimerTaskEnable(btn->holdTimerTask);
            }
            break;
        case PRESSED:
            // If button is released
            if (ButtonIsReleased(btn)) {
                btn->currentState   = RELEASED;
                btn->lastActionTime = millisecondsElasped();
                // If a hold timer tasks exists, disable and reset it.
                if (btn->holdTimerTask) {
                    TimerTaskDisable(btn->holdTimerTask);
                    TimerTaskReset(btn->holdTimerTask);
                }
                ButtonActionFlagSetPress(btn);
                if (btn->eventPress)
                    btn->eventPress();
            }
            break;
        case HELD:
            if (ButtonIsReleased(btn)) {
                btn->currentState   = RELEASED;
                btn->lastActionTime = millisecondsElasped();
                ButtonActionFlagSetClear(btn);
                TimerTaskDisable(btn->holdTimerTask);
                TimerTaskReset(btn->holdTimerTask);
            }
            break;
        default:
            btn->currentState = RELEASED;
            break;
    }
}

/**
 * @brief The generic callback for hold events, triggered by a buttons hold timer task.
 * If button is null returns immediately.
 * 
 * If button is not null then updates the state to HELD, sets the action flag to FLAG_HELD
 * and calls the eventHeld callback, if set.
 * 
 * @param button 
 */
static void enterHeldState(void *button) {
    if (!button)
        return;

    Button *btn = (Button *)button;

    btn->currentState = HELD;
    ButtonActionFlagSetHold(btn);
    if (btn->eventHeld)
        btn->eventHeld();
}


/**
 * @brief Clears the action flags for all buttons.
 * 
 */
void ButtonClearAllActionFlags() {
    for (int i = 0; i < buttonUsage; i++) {
        ButtonActionFlagSetClear(&buttons[i]);
    }
}

/**
 * @brief Clears the Button's Action flag, setting to FLAG_CLEAR.
 * 
 * @param btn Button to set flag on. If null does nothing.
 */
static inline void ButtonActionFlagSetClear(Button *btn) {
    if (btn)
        btn->actionFlag = FLAG_CLEAR;
}

/**
 * @brief Clears all Buttons action flags, sets their states to RELEASED and disables hold timers.
 * Only used when entering alarm mode to try help prevent unwanted button events.
 */
void ButtonClearFlagsAndForceToReleased() {
    for (int i = 0; i < buttonUsage; i++) {
        ButtonActionFlagSetClear(&buttons[i]);
        buttons[i].currentState = RELEASED;
        if(buttons[i].holdTimerTask)
            TimerTaskDisable(buttons[i].holdTimerTask);

    }
}

/**
 * @brief Sets the provided Button's action flag to FLAG_PRESSED.
 * 
 * @param btn Button to set flag of. If null does nothing.
 */
static inline void ButtonActionFlagSetPress(Button *btn) { if (btn) btn->actionFlag = FLAG_PRESSED; }

/**
 * @brief Sets the provided Button's action flag to FLAG_HELD.
 * 
 * @param btn Button to set flag of. If null does nothing.
 */
static inline void ButtonActionFlagSetHold(Button *btn) { if (btn) btn->actionFlag = FLAG_HELD; }

/**
 * @brief Gets the provided Button's action flag.
 * 
 * @param btn Button to operate upon.
 * @return ButtonActionFlag The action flag of btn. If btn is null, returns FLAG_UNKNOWN.
 */
inline ButtonActionFlag ButtonReadFlag(Button *btn) {
    if (!btn)
        return FLAG_UNKNOWN;
    return btn->actionFlag;
}

/**
 * @brief Updates a buttons to refresh their internal states, calling any callbacks if required.
 */
inline void ButtonUpdateAll() {
    for (int i = 0; i < buttonUsage; i++) {
        ButtonUpdate(&buttons[i]);
    }
}

/**
 * @brief Determines whether a button is pressed or not.
 * 
 * @param btn The button to check the state of.
 * @return true If the button is pressed.
 * @return false If the button is not pressed.
 */
bool ButtonIsPressed(Button *btn) { return !ButtonIsReleased(btn); }

/**
 * @brief Determines whether a button is released or not.
 * 
 * @param btn The button to check the state of.
 * @return true If the button is released.
 * @return false If the button is not released.
 */
bool ButtonIsReleased(Button *btn) { return (*btn->inRegister & (1 << btn->buttonPin)); }


/**
 * @brief Used to debounce button events.
 * 
 * @param btn Button to verify timing of.
 * @return true If the current time (millisecondsElasped()) minus the time the button was last pressed 
 * (lastActionTime) is greater than BUTTON_CHANGE_DELAY_MS.
 * @return false If the current time (millisecondsElasped()) minus the time the button was last pressed 
 * (lastActionTime) is less than or equal to BUTTON_CHANGE_DELAY_MS.
 */
static bool ButtonIsTimingCorrect(Button *btn) {
    if (!btn)
        return false;

    uint64_t delta = millisecondsElasped() - btn->lastActionTime;

    return delta > BUTTON_CHANGE_DELAY_MS;
}