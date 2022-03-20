#include "fsm.h"

// Maybe put into a fsm.c file. This implementation is generic enough that we dont
// need it specified here.
void FSMUpdate(FSM_TRANSITION_TABLE *table, void (*callback)(TransitionCallback))
{
    uint8_t matchesFound = 0;
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
            matchesFound++;
            if (transition.trigger())
            {
                transition.action();
                table->currentState = transition.nextState;
                
                TransitionCallback response = {TRANSITIONED, table->currentState};
                callback(response);
                
                break;
            } else {
                TransitionCallback response = {NOT_TRIGGERED, 0};
                callback(response);
            }
        }
    }

    TransitionCallback response = {matchesFound ? MATCHES : NO_MATCH, matchesFound};
    callback(response);
}

void noAction() {}

bool noEvent() {
    return true;
}

bool noContinue() {
    return false;
}