#include "flag.h"

Flag Flag_Create(void (*callback)(void *arg)) {
    return (Flag){false, true, callback, NULL};
}

bool Flag_RunIfSet(Flag *f) {
    if (!f)
        return false;

    if (f->set && f->enabled) {
        f->set = false;
        f->callback(f->callbackArg);
        return true;
    }
    return false;
}

void Flag_Set(Flag *f) {
    if (f) {
        f->set = true;
    }
}

void Flag_Clear(Flag *f) {
    if (f)
        f->set = false;
}

void Flag_Enable(Flag *f) {
    if (f) {
        f->enabled = true;
    }
}

void Flag_Disable(Flag *f) {
    if (f) {
        f->enabled = false;
    }
}
