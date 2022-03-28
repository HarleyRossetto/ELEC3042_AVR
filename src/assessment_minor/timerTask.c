#include "timerTask.h"

static TimerTask timerTasks[MAX_TIMERTASKS];
static uint8_t timerTaskUsage = 0;

TimerTask *TimerTaskCreate(uint64_t p, Callback callback, CallbackArg callbackArg, bool enable, bool oneShot) {
      if (timerTaskUsage > MAX_TIMERTASKS)
        return 0;
        
    timerTasks[timerTaskUsage] = (TimerTask){p, callback, callbackArg, 0, enable, oneShot};
    return &timerTasks[timerTaskUsage++];
}

inline void TimerTaskEnable(TimerTask *t) {
    if (t)
        t->enabled = true;
}

inline void TimerTaskDisable(TimerTask *t) {
    if (t)
        t->enabled = false;
}

inline void TimerTaskReset(TimerTask *t) {
    if (t)
        t->elaspedTime = 0;
}

void TimerTaskUpdate(uint64_t delta) {
    for (int i = 0; i < timerTaskUsage; i++) {
        TimerTask *t = &timerTasks[i];
        if (t->enabled) {
            t->elaspedTime += delta;

            if (t->elaspedTime >= t->period) {
                t->eventCallback(t->arg);
                t->elaspedTime = 0;
                // If a 1-shot timer, disable after execution.
                if (t->oneShot)
                    t->enabled = false;
            }
        }
    }
}

inline TimerTask *TimerTaskGetTimers() { return timerTasks; }
