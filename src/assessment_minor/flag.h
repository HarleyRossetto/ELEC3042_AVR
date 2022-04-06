#ifndef FLAG_H
#define FLAG_H

#include "bool.h"
#include "null.h"

typedef struct {
    bool set;
    bool enabled;
    void (*callback)(void *arg);
    void *callbackArg;
} Flag;

Flag Flag_Create(void(*callback)(), void*callbackArg);
bool Flag_RunIfSet(Flag *f);
void Flag_Set(Flag *f);
bool Flag_IsSet(Flag *f);
void Flag_Clear(Flag *f);
void Flag_Toggle(Flag *f);
void Flag_Enable(Flag *f);
void Flag_Disable(Flag *f);

#endif //FLAG_H