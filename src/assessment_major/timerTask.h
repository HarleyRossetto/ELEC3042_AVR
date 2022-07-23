/**
 * @file timerTask.h
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

#ifndef TIMERTASK_H
#define TIMERTASK_H

#include "bool.h"
#include "stdint.h"

#define MAX_TIMERTASKS 30

typedef void (*Callback)(void *arg);
typedef void *CallbackArg;

typedef struct {
    uint64_t period;
    Callback eventCallback;
    CallbackArg arg;
    uint64_t elaspedTime;
    bool enabled;
    bool oneShot;
} TimerTask;

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
TimerTask *TimerTaskCreate(uint64_t p, Callback callback, CallbackArg callbackArg, bool enable, bool oneShot);

/**
 * @brief Enables the timer task.
 * 
 * @param t Task to enable, if null does nothing.
 */
void TimerTaskEnable(TimerTask *t);

/**
 * @brief Disables the timer task.
 * 
 * @param t Task to disable, if null does nothing.
 */
void TimerTaskDisable(TimerTask *t);

/**
 * @brief Resets the timer task elasped time. (i.e. sets elaspedTime to 0)
 * 
 * @param t Task to reset, if null does nothing.
 */
void TimerTaskReset(TimerTask *t);

void TimerTaskRestart(TimerTask *t);

/**
 * @brief Updates all timer tasks which are currently enabled.
 * If their elasped time is greater than or equal to their period then will activate their event
 * callback (if not null); the elaspedTime will be reset to 0 and if in oneshot mode the timer 
 * will be disabled.
 *
 * @param delta Time in milliseconds since the timers were last updated.
 */
void TimerTaskUpdate(uint64_t delta);

void TimerTaskSetPeriod(TimerTask *timer, uint64_t period);

/**
 * @brief Provides a pointer the all internal timer tasks.
 * 
 * @return TimerTask* Pointer to timer task array.
 */
TimerTask *TimerTaskGetTimers();

#endif // TIMERTASK_H