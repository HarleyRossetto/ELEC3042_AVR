#ifndef FSM_H
#define FSM_H

#include "stdint.h"
#include "bool.h"

typedef BOOL (*FSM_TRIGGER)();
typedef void (*FSM_ACTION)();

typedef enum
{
    DISPLAY_HH_MM, DISPLAY_MM_SS, SET_TIME_MODE_HR, SET_TIME_MODE_MIN
} FSM_STATE;

typedef struct
{
    FSM_STATE currentState;
    FSM_TRIGGER trigger;
    FSM_ACTION action;
    FSM_STATE nextState;
} FSM_TRANSITION;


#define FSM_TRANSITION_MAX 8
typedef struct 
{
    FSM_STATE currentState;
    FSM_TRANSITION transitions[FSM_TRANSITION_MAX];
} FSM_TRANSITION_TABLE;

void noAction() { }

#endif
