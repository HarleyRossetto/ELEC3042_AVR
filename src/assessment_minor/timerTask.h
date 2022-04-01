#ifndef TIMERTASK_H
#define TIMERTASK_H

#include "bool.h"
#include "stdint.h"

#define MAX_TIMERTASKS 20

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

TimerTask *TimerTaskCreate(uint64_t p, Callback callback, CallbackArg callbackArg, bool enable, bool oneShot);
void TimerTaskEnable(TimerTask *t);
void TimerTaskDisable(TimerTask *t);
void TimerTaskReset(TimerTask *t);
void TimerTaskUpdate(uint64_t delta);
TimerTask *TimerTaskGetTimers();

#endif // TIMERTASK_H