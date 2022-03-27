#ifndef TICKABLE_H
#define TICKABLE_H

#include "bool.h"
#include "stdint.h"

#define MAX_TICKABLES 10

typedef void (*TickableCallback)(void *arg);
typedef void *CallbackArg;

typedef struct {
    uint64_t period;
    TickableCallback eventCallback;
    CallbackArg arg;
    uint64_t elaspedTime;
    bool enabled;
    bool oneShot;
} Tickable;

Tickable *TickableCreate(uint64_t p, void (*e)(), bool enable, bool oneShot);
void TickableEnable(Tickable *t);
void TickableDisable(Tickable *t);
void TickableReset(Tickable *t);
void TickableUpdate(uint64_t delta);
Tickable *TickableList();

#endif // TICKABLE_H