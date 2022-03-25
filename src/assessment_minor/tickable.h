#ifndef TICKABLE_H
#define TICKABLE_H

#include "stdint.h"

#define MAX_TICKABLES 6

typedef struct {
    uint64_t period;
    void (*eventCallback)();
    uint64_t elaspedTime;
} Tickable;

Tickable TickableCreate(uint64_t p, void (*e)());
void TickableUpdate(uint64_t delta);
Tickable* TickableList();

#endif // TICKABLE_H