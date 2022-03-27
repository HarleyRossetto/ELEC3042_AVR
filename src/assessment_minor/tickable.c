#include "tickable.h"

static Tickable tickables[MAX_TICKABLES];
static uint8_t tickablesUsage = 0;

Tickable *TickableCreate(uint64_t p, void (*e)(), bool enable, bool oneShot) {
      if (tickablesUsage > MAX_TICKABLES)
        return 0;
        
    tickables[tickablesUsage] = (Tickable){p, e, 0, 0, enable, oneShot};
    return &tickables[tickablesUsage++];
}

inline void TickableEnable(Tickable *t) {
    if (t)
        t->enabled = true;
}

inline void TickableDisable(Tickable *t) {
    if (t)
        t->enabled = false;
}

inline void TickableReset(Tickable *t) {
    if (t)
        t->elaspedTime = 0;
}

void TickableUpdate(uint64_t delta) {
    for (int i = 0; i < tickablesUsage; i++) {
        Tickable *t = &tickables[i];
        if (t->enabled) {
            t->elaspedTime += delta;

            if (t->elaspedTime >= t->period) {
                t->eventCallback(t->arg);
                t->elaspedTime = 0;
                // If a 1-shot tickable, disable after execution.
                if (t->oneShot)
                    TickableDisable(t);
            }
        }
    }
}

inline Tickable *TickableList() { return tickables; }
