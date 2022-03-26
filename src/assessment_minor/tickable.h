#ifndef TICKABLE_H
#define TICKABLE_H

#include "stdint.h"
#include "bool.h"

#define MAX_TICKABLES 6

typedef struct {
    uint64_t period;
    void (*eventCallback)();
    uint64_t elaspedTime;
    bool enabled;
    bool oneShot;
} Tickable;

Tickable *TickableCreate(uint64_t p, void (*e)(), bool enable, bool oneShot);
void TickableEnable(Tickable *t);
void TickableDisable(Tickable *t);
void TickableReset(Tickable *t);
void TickableUpdate(uint64_t delta);
Tickable* TickableList();

#endif // TICKABLE_H