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
#include "sensor.h"

extern Sensor sensor0;
extern Sensor sensor1;
extern Sensor sensor2;
extern Sensor sensor3;
extern Sensor sensor4;
extern Sensor sensor5;
extern Sensor sensor6;

FSM_STATE state_before_amber;
FSM_STATE state_after_red;

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

                const FSM_STATE initial = table->currentState;

                // If the current state and next state is the same, bail out of here.
                if (table->currentState == transition.nextState())
                    return;

                transition.action();

                // if no red or amber store next state in tempory, move to amber, then red, then move to tempory value.
                if (table->currentState != INTERSECTION_AMBER && table->currentState != INTERSECTION_RED) {
                    state_before_amber  = transition.currentState;
                    state_after_red     = transition.nextState();
                    table->currentState = INTERSECTION_AMBER; // Force to amber

                } else {
                    table->currentState = transition.nextState();
                }

                if (initial != table->currentState)
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
FSM_STATE STATE_AMBER() { return INTERSECTION_AMBER; }
FSM_STATE STATE_RED() { return INTERSECTION_RED; }


FSM_STATE STATE_AFTER_RED() { return state_after_red; }