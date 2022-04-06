/**
 * @file flag.h
  * @author Harley Rossetto (44618883@students.mq.edu.au)
 * @brief Abstractions for boolean values with callbacks.
 * Each flag as a value (set), an enabled state, and callback and callback arg pointers.
 * @version 0.1
 * @date 2022-04-06
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef FLAG_H
#define FLAG_H

#include "bool.h"
#include "null.h"

typedef struct {
    bool set;
    bool enabled;
    void (*callback)(void *arg);
    void *callbackArg;
} Flag;

/**
 * @brief Creates a flag with the provided callback and argument pointers, setting enabled to true,
 * and current value to false.
 * 
 * @param callback Callback function to call when flag is set and checked, see Flag_RunIfSet(..)
 * @param callbackArg Callback argument to pass to the callback function.
 * @return Flag new struct Flag.
 */
Flag Flag_Create(void(*callback)(), void*callbackArg);

/**
 * @brief Checks if the Flag f is set, and if so calls it's callback (if set) and clears it's internal
 * set flag.
 * 
 * @param f Flag to check, if null immediately returns.
 * @return true If the flag was set.
 * @return false If the flag was not set or null.
 */
bool Flag_RunIfSet(Flag *f);

/**
 * @brief Sets the Flag f to true. If null, does nothing.
 * 
 * @param f Flag to set.
 */
void Flag_Set(Flag *f);

/**
 * @brief Checks if a flag is set.
 * 
 * @param f Flag to check
 * @return true If flag f is not null and is set.
 * @return false If flag f is null or is not net.
 */
bool Flag_IsSet(Flag *f);

/**
 * @brief Sets the Flag f to false. If null, does nothing.
 * 
 * @param f Flag to clear.
 */
void Flag_Clear(Flag *f);

/**
 * @brief Toggles the flags state.
 * 
 * @param f Flag to toggle.
 */
void Flag_Toggle(Flag *f);

#endif //FLAG_H