#ifndef FSM_H
#define FSM_H

#include "stdint.h"
#include "bool.h"

typedef bool (*FSM_TRIGGER)();
typedef void (*FSM_ACTION)();

#define FSM_STATE_COUNT 4
typedef enum FSM_STATES
{
    DISPLAY_HH_MM, DISPLAY_MM_SS, SET_TIME_MODE_HR, SET_TIME_MODE_MIN
} FSM_STATE;

typedef struct FSM_TRANS
{
    FSM_STATE currentState;
    FSM_TRIGGER trigger;
    FSM_ACTION action;
    FSM_STATE nextState;
} FSM_TRANSITION;

#define FSM_TRANSITION_MAX 9
typedef struct FSM_TRANS_TBL
{
    FSM_STATE currentState;
    FSM_TRANSITION transitions[FSM_TRANSITION_MAX];
} FSM_TRANSITION_TABLE;

typedef enum
{
    TRIGGERED,
    NOT_TRIGGERED,
    ACTION,
    TRANSITIONED,
    NO_MATCH,
    MATCHES
} TransitionCallbackReason;

typedef enum
{
    NO_STATE_CHANGE,
    STATE_CHANGE
} FSMUpdateResult;

typedef struct {
    TransitionCallbackReason reason;
    uint8_t data;
} TransitionCallback;

void
noAction();
bool noEvent();
bool noContinue();
FSMUpdateResult FSMUpdate(FSM_TRANSITION_TABLE *table);

#endif
