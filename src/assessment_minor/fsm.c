#include "fsm.h"

// Maybe put into a fsm.c file. This implementation is generic enough that we dont
// need it specified here.
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
                table->currentState = transition.nextState;
                updateResult = STATE_CHANGE;
                break;
            }
        }
    }

    return updateResult;
}

void noAction() {}

bool noEvent() {
    return true;
}

bool noContinue() {
    return false;
}