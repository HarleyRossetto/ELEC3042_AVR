/**
 * @file button.h
  * @author Harley Rossetto (44618883@students.mq.edu.au)
 * @brief Button abstraction types and functions.
 * Each button is stored as a struct which maintains each of their own states.
 * @version 0.1
 * @date 2022-04-06
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef BUTTON_H
#define BUTTON_H

#include "stdint.h"
#include "bool.h"
#include "avr/io.h"
#include "timerTask.h"
#include "iotypes.h"

typedef enum
{
    RELEASED, PRESSED, HELD
} ButtonState;

#define BUTTON_CHANGE_DELAY_MS 15

// Action flags are close to reflecting ButtonState enumeration values, however serve a difference purpose.
// The ButtonState is internally by the button to manage it's state changes, whilst ButtonActionFlag is used
// externally.
// Flags are set when a buttons state is changed and are used to determine which button event to respond to.
// There flags are reset after a major state change to avoid subsequent state changes responding to the same event.
typedef enum { FLAG_CLEAR, FLAG_PRESSED, FLAG_HELD, FLAG_UNKNOWN } ButtonActionFlag;

typedef struct {
    Port inRegister;
    Pin buttonPin;
    ButtonState currentState;       // Internal Button State
    uint64_t lastActionTime;
    void(*eventPress)();
    void(*eventHeld)();
    TimerTask *holdTimerTask;
    ButtonActionFlag actionFlag;    // Flags indicate button state, but are cleared upon a state transition.
} Button;

#define MAX_BUTTONS 3
typedef Button ButtonArray[MAX_BUTTONS];

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
Button *ButtonCreate(Port ddr, Port port, Port, Pin  pin, void (*pressEvent)(), void (*holdEvent)(), bool attachInterrupt);

/**
 * @brief Determines whether a button is pressed or not.
 * 
 * @param btn The button to check the state of.
 * @return true If the button is pressed.
 * @return false If the button is not pressed.
 */
bool ButtonIsPressed(Button *btn);

/**
 * @brief Determines whether a button is released or not.
 * 
 * @param btn The button to check the state of.
 * @return true If the button is released.
 * @return false If the button is not released.
 */
bool ButtonIsReleased(Button *btn);

/**
 * @brief Updates a Buttons state (Pressed, Released, Held). 
 * If *btn null or debounce time insufficient function does nothing and returns.
 * @param btn Button to update. 
 */
void ButtonUpdate(Button *btn);

/**
 * @brief Updates a buttons to refresh their internal states, calling any callbacks if required.
 */
void ButtonUpdateAll();

/**
 * @brief Provides a reference to the internal button array in the provided parameter.
 * 
 * @param btns A pointer to a ButtonArray in which to store a reference to the internal buttons array in.
 */
void ButtonsGetAll(ButtonArray *btns);

/**
 * @brief Clears the action flags for all buttons.
 * 
 */
void ButtonClearAllActionFlags();

/**
 * @brief Clears all Buttons action flags, sets their states to RELEASED and disables hold timers.
 * Only used when entering alarm mode to try help prevent unwanted button events.
 */
void ButtonClearFlagsAndForceToReleased();

/**
 * @brief Gets the provided Buttons action flag.
 * 
 * @param btn Button to operate upon.
 * @return ButtonActionFlag The action flag of btn. If btn is null, returns FLAG_UNKNOWN.
 */
ButtonActionFlag ButtonReadFlag(Button *btn);

#endif //BUTTON_H