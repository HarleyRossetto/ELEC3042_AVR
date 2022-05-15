/**
 * @file fsm.c
 * @author Harley Rossetto (44618883@students.mq.edu.au)
 * @brief Finite State Machine abstractions.
 * @version 0.1
 * @date 2022-04-06
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "fsm.h"

/**
 * @brief Heart of the Finite State Machines functioning.
 * FSMUpdate iterates through all transitions listed in the transition table.
 * If any transition state matches the systems current state then we will check if it's transition 
 * trigger is active, if it is we will call the transitions action function before update the system state.
 * 
 * @param table 
 * @return FSMUpdateResult 
 */
FSMUpdateResult FSMUpdate(FSM_TRANSITION_TABLE *table)
{
    FSMUpdateResult updateResult = NO_STATE_CHANGE;
    /**
     * Iterate overall potential transitions.
     * If any transition matches the current state, check its trigger(s).
     * If they have been fired, execute the transitions action and move to the next state.
     *
     */
    for (int index = 0; index < FSM_TRANSITION_MAX; index++)
    {
        FSM_TRANSITION transition = table->transitions[index];

        if (transition.currentState == table->currentState)
        {
            if (transition.trigger())
            {
                transition.action();
                table->currentState = transition.nextState();
                updateResult = STATE_CHANGE;
                break;
            }
        }
    }

    return updateResult;
}

void noAction() {}

bool noTrigger() {
    return true;
}

FSM_STATE STATE_BROADWAY() { return BROADWAY; }
FSM_STATE STATE_BROADWAY_SOUTHBOUND() { return BROADWAY_SOUTHBOUND; }
FSM_STATE STATE_BROADWAY_TURN_AND_PEDESTRIANS() { return BROADWAY_TURN_AND_PEDESTRIANS; }
FSM_STATE STATE_LITTLE_STREET() { return LITTLE_STREET; }
FSM_STATE STATE_BROADWAY_STRAIGHT_AND_LITTLE_STREET_PEDESTRIAN() { return BROADWAY_STRAIGHT_AND_LITTLE_STREET_PEDESTRIAN; }