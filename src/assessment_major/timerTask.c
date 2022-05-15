/**
 * @file timerTask.c
 * @author Harley Rossetto (44618883@students.mq.edu.au)
 * @brief Abstractions for creating and managing timed tasks.
 * Tasks can be scheduled to run after period (defined in milliseconds), either once or on a recurring basis.
 * Tasks will calls their callbacks when they reach their trigger period, stopping if oneShot, or resetting if not.
 * @version 0.1
 * @date 2022-04-06
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "timerTask.h"

static TimerTask timerTasks[MAX_TIMERTASKS];
static uint8_t timerTaskUsage = 0;

#define INITIAL_ELASPED_TIME 0

/**
 * @brief Creates a timer task and stores it internally whilst returning a reference to the task.
 * 
 * @param p The timers period, i.e. the time at which it will execute it's callback. This value is not 
 * the specific time in the future it will run at, rather the period between now and when we want it to run.
 * If the current time in millis is 2355882 and we want it to run in 15 seconds, then p would be 15000.
 * @param callback Function to call when the timer elapses.
 * @param callbackArg Argument to pass to callback function.
 * @param enable Whether or not to enable the timer initially.
 * @param oneShot Whether the timer is to run in oneshot mode (if true) or recurring (if false).
 * @return TimerTask* Reference to the newly created timer task.
 */
TimerTask *TimerTaskCreate(uint64_t p, Callback callback, CallbackArg callbackArg, bool enable, bool oneShot) {
      if (timerTaskUsage > MAX_TIMERTASKS)
        return 0;
        
    timerTasks[timerTaskUsage] = (TimerTask){p, callback, callbackArg, INITIAL_ELASPED_TIME, enable, oneShot};
    return &timerTasks[timerTaskUsage++];
}

/**
 * @brief Enables the timer task.
 * 
 * @param t Task to enable, if null does nothing.
 */
inline void TimerTaskEnable(TimerTask *t) {
    if (t)
        t->enabled = true;
}

/**
 * @brief Disables the timer task.
 * 
 * @param t Task to disable, if null does nothing.
 */
inline void TimerTaskDisable(TimerTask *t) {
    if (t)
        t->enabled = false;
}

/**
 * @brief Resets the timer task elasped time. (i.e. sets elaspedTime to 0)
 * 
 * @param t Task to reset, if null does nothing.
 */
inline void TimerTaskReset(TimerTask *t) {
    if (t)
        t->elaspedTime = 0;
}

void TimerTaskSetPeriod(TimerTask *timer, uint64_t period) {
    if (!timer)
        return;

    timer->period = period;
}

/**
 * @brief Updates all timer tasks which are currently enabled.
 * If their elasped time is greater than or equal to their period then will activate their event
 * callback (if not null); the elaspedTime will be reset to 0 and if in oneshot mode the timer 
 * will be disabled.
 *
 * @param delta Time in milliseconds since the timers were last updated.
 */
void TimerTaskUpdate(uint64_t delta) {
    // GO through all timers
    for (int i = 0; i < timerTaskUsage; i++) {
        TimerTask *t = &timerTasks[i];
        // If enabled
        if (t->enabled) {
            t->elaspedTime += delta;

            // Time period has elasped
            if (t->elaspedTime >= t->period) {
                //Call callback function if it exists
                if (t->eventCallback)
                    t->eventCallback(t->arg);

                // Reset time.
                t->elaspedTime = 0;

                // If a 1-shot timer, disable after execution.
                if (t->oneShot)
                    t->enabled = false;
            }
        }
    }
}

/**
 * @brief Provides a pointer the all internal timer tasks.
 * 
 * @return TimerTask* Pointer to timer task array.
 */
inline TimerTask *TimerTaskGetTimers() { return timerTasks; }
