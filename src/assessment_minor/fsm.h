/**
 * @file fsm.h
 * @author Harley Rossetto (44618883@students.mq.edu.au)
 * @brief Finite State Machine abstractions.
 * @version 0.1
 * @date 2022-04-06
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef FSM_H
#define FSM_H

#include "stdint.h"
#include "bool.h"

// Typedef for named transition trigger callback function.
typedef bool (*FSM_TRIGGER_FUNC)();
// Typedef for trigger function return, used to annotate purpose of function.
typedef bool FSMTrigger;
// Typedef for named transition action callback function.
typedef void (*FSM_ACTION_FUNC)();
// Typedef for action function return, used to annotate purpose of function.
typedef void FSMAction;

// All the core states in the system
typedef enum 
{
    DISPLAY_HH_MM, DISPLAY_MM_SS, SET_TIME_MODE_HR, SET_TIME_MODE_MIN, DISPLAY_ALARM, ALARM_SET_HR, ALARM_SET_MIN, NUMBER_OF_STATES
} FSM_STATE;
// Maximum number of states, use a dirty trick to make this easy to keep track of.
// Just need to keep NUMBER_OF_STATES at end of FSM_STATE enumeration.
#define FSM_STATE_COUNT NUMBER_OF_STATES

// Prototypes for state callbacks
FSM_STATE STATE_HH_MM();
FSM_STATE STATE_MM_SS();
FSM_STATE STATE_MODE_HR();
FSM_STATE STATE_MODE_MIN();
FSM_STATE STATE_DISP_ALARM();
FSM_STATE STATE_ALARM_HR();
FSM_STATE STATE_ALARM_MIN();

/**
 * @brief FSM Transition structure 
 * Requires currentState to determine which state this transition belongs to.
 * 
 * Has a trigger function, if it doesn't require a specialised one, must use noTrigger() function.
 * This will enable the transition to immediately execute, used where there may be intermediary states/transitions,
 * which require no other inputs.
 * 
 * The action refers to a callback function to call when the transition is triggered. If this is not required
 * it should be set to noAction().
 * 
 * Finally has a callback to a function which returns the next state the transition will lead to if triggered,
 * after the action as been called.
 * This is a callback as a dirty trick to allow the alarm set mode can have a programable next state which
 * is determined before it transitions. Otherwise if a constant would always return on only one state.
 */
typedef struct FSM_TRANS
{
    FSM_STATE currentState;
    FSM_TRIGGER_FUNC trigger;
    FSM_ACTION_FUNC action;
    FSM_STATE (*nextState)();
} FSM_TRANSITION;

#define FSM_TRANSITION_MAX 30
/**
 * @brief FSMTransition Table: this is where the FSM's state is maintained.
 * 
 */
typedef struct FSM_TRANS_TBL
{
    FSM_STATE currentState;
    FSM_TRANSITION transitions[FSM_TRANSITION_MAX];
} FSM_TRANSITION_TABLE;

/**
 * @brief Enumeration type returned by the FSMUpdate(..) function which alerts the caller as to whether
 * or not a change of state has occured.
 * 
 */
typedef enum
{
    NO_STATE_CHANGE,
    STATE_CHANGE
} FSMUpdateResult;

void noAction();
bool noTrigger();
FSMUpdateResult FSMUpdate(FSM_TRANSITION_TABLE *table);

#endif //FSM_H