#ifndef FSM_H
#define FSM_H

#include "stdint.h"
#include "bool.h"

typedef bool (*FSM_TRIGGER_FUNC)();
typedef bool FSMTrigger;
typedef void (*FSM_ACTION_FUNC)();
typedef void FSMAction;

#define FSM_STATE_COUNT 4
typedef enum FSM_STATES
{
    DISPLAY_HH_MM, DISPLAY_MM_SS, SET_TIME_MODE_HR, SET_TIME_MODE_MIN
} FSM_STATE;

typedef struct FSM_TRANS
{
    FSM_STATE currentState;
    FSM_TRIGGER_FUNC trigger;
    FSM_ACTION_FUNC action;
    FSM_STATE nextState;
} FSM_TRANSITION;

#define FSM_TRANSITION_MAX 10
typedef struct FSM_TRANS_TBL
{
    FSM_STATE currentState;
    FSM_TRANSITION transitions[FSM_TRANSITION_MAX];
} FSM_TRANSITION_TABLE;

typedef enum
{
    NO_STATE_CHANGE,
    STATE_CHANGE
} FSMUpdateResult;

void
noAction();
bool noEvent();
bool noContinue();
FSMUpdateResult FSMUpdate(FSM_TRANSITION_TABLE *table);

#endif
