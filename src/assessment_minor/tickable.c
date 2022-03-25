#include "tickable.h"

static Tickable tickables[MAX_TICKABLES];
static uint8_t tickablesUsage = 0;

Tickable TickableCreate(uint64_t p, void (*e)()) {
    tickables[tickablesUsage] = (Tickable){p, e, 0};
    return tickables[tickablesUsage++];
}

void TickableUpdate(uint64_t delta) {
    for (int i = 0; i < tickablesUsage; i++) {
        Tickable *t = &tickables[i];
        t->elaspedTime += delta;

        if (t->elaspedTime >= t->period) {
            t->eventCallback();
            t->elaspedTime = 0;
        }
    }
}

Tickable* TickableList() {
    return tickables;
}
