#include "tickable.h"

static Tickable tickables[MAX_TICKABLES];
static uint8_t tickablesUsage = 0;

Tickable *TickableCreate(uint64_t p, void (*e)(), bool enable) {
    tickables[tickablesUsage] = (Tickable){p, e, 0, enable};
    return &tickables[tickablesUsage++];
}

void TickableEnable(Tickable *t) {
    if (t)
        t->enabled = true;
}

void TickableDisable(Tickable *t) {
    if (t)
        t->enabled = false;
}

void TickableReset(Tickable *t) {
    if (t)
        t->elaspedTime = 0;
}

void TickableUpdate(uint64_t delta) {
    for (int i = 0; i < tickablesUsage; i++) {
        Tickable *t = &tickables[i];
        if (t->enabled) {
            t->elaspedTime += delta;
    
            if (t->elaspedTime >= t->period) {
                t->eventCallback();
                t->elaspedTime = 0;
            }
        }
    }
}

Tickable* TickableList() {
    return tickables;
}
