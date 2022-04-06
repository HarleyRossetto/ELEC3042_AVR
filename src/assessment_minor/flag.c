/**
 * @file flag.c
  * @author Harley Rossetto (44618883@students.mq.edu.au)
 * @brief Abstractions for boolean values with callbacks.
 * Each flag as a value (set), an enabled state, and callback and callback arg pointers.
 * @version 0.1
 * @date 2022-04-06
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "flag.h"

/**
 * @brief Creates a flag with the provided callback and argument pointers, setting enabled to true,
 * and current value to false.
 * 
 * @param callback Callback function to call when flag is set and checked, see Flag_RunIfSet(..)
 * @param callbackArg Callback argument to pass to the callback function.
 * @return Flag new struct Flag.
 */
Flag Flag_Create(void (*callback)(void *arg), void *callbackArg) {
    return (Flag){false, true, callback, callbackArg};
}

/**
 * @brief Checks if the Flag f is set, and if so calls it's callback (if set) and clears it's internal
 * set flag.
 * 
 * @param f Flag to check, if null immediately returns.
 * @return true If the flag was set.
 * @return false If the flag was not set or null.
 */
bool Flag_RunIfSet(Flag *f) {
    if (!f)
        return false;

    if (f->set && f->enabled) {
        f->set = false;
        if (f->callback)
            f->callback(f->callbackArg);
        return true;
    }
    return false;
}

/**
 * @brief Sets the Flag f to true. If null, does nothing.
 * 
 * @param f Flag to set.
 */
void Flag_Set(Flag *f) {
    if (f) {
        f->set = true;
    }
}

/**
 * @brief Checks if a flag is set.
 * 
 * @param f Flag to check
 * @return true If flag f is not null and is set.
 * @return false If flag f is null or is not net.
 */
bool Flag_IsSet(Flag *f) {
    return (f && f->set);
}

/**
 * @brief Sets the Flag f to false. If null, does nothing.
 * 
 * @param f Flag to clear.
 */
void Flag_Clear(Flag *f) {
    if (f)
        f->set = false;
}

/**
 * @brief Toggles the flags state.
 * 
 * @param f Flag to toggle.
 */
void Flag_Toggle(Flag *f) {
    if (f)
        f->set = !f->set;
}
