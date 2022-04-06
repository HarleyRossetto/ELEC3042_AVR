#include "debug_display.h"

bool transitioned = false;
uint8_t matches   = 0;
bool triggered    = false;

void displayFunctionIncHold() {
    ButtonArray *btns = NULL;
    GetButtons(btns);

    TimerTask *incTick = btns[2]->holdTimerTask;

    displayData.data[SEG_FAR_RIGHT] = mapChar((incTick->elaspedTime >> 4) & 0xF); // Far Right
    displayData.data[SEG_RIGHT]     = mapChar((incTick->elaspedTime >> 8) & 0xF); // Centre Right
    displayData.data[SEG_LEFT]      = mapChar(incTick->oneShot & 1);              // Center Left
    displayData.data[SEG_FAR_LEFT]  = mapChar(incTick->enabled & 1);              // Far Left
}

volatile uint8_t isrTime    = 0;
volatile uint8_t isrTimeEnd = 0;
void displayFunctionISRTime() {
    // uint64_t delta = isrTimeEnd - isrTime;
    int8_t delta = isrTimeEnd - isrTime;

    displayData.data[SEG_FAR_RIGHT] = mapChar(delta & 0xF);         // Far Right
    displayData.data[SEG_RIGHT]     = mapChar((delta >> 4) & 0xF);  // Centre Right
    displayData.data[SEG_LEFT]      = mapChar((delta >> 8) & 0xF);  // Center Left
    displayData.data[SEG_FAR_LEFT]  = mapChar((delta >> 12) & 0xF); // Far Left
}

void displayFunctionTwo8BitReg(volatile uint8_t *r1, volatile uint8_t *r2) {
    displayData.data[SEG_FAR_RIGHT] = mapChar(*r2 & 0xF);        // Far Right
    displayData.data[SEG_RIGHT]     = mapChar((*r2 >> 4) & 0xF); // Centre Right
    displayData.data[SEG_LEFT]      = mapChar(*r1 & 0xF);        // Center Left
    displayData.data[SEG_FAR_LEFT]  = mapChar((*r1 >> 4) & 0xF); // Far Left
}

void displayFunction8BitReg(volatile uint8_t *r1) { displayFunctionTwo8BitReg(r1, 0); }

void displayFunctionPCICR() { displayFunction8BitReg(&PCICR); }

void displayFunctionDDRC() { displayFunctionTwo8BitReg(&DDRC, &PORTC); }

void displayFunctionButtonStates() {
    ButtonArray *btns = NULL;
    GetButtons(btns);
    displayData.data[SEG_FAR_RIGHT] = mapChar(btns[2]->currentState); // Far Right
    displayData.data[SEG_RIGHT]     = mapChar(btns[1]->currentState); // Centre Right
    displayData.data[SEG_LEFT]      = mapChar(btns[0]->currentState); // Center Left
    displayData.data[SEG_FAR_LEFT]  = mapChar(BLANK);                 // Far Left
}

void displayFunctionCurrentState() {
    displayData.data[SEG_FAR_RIGHT] = mapChar(stateMachinePtr->currentState); // Far Right -> Current State Index
    displayData.data[SEG_RIGHT]     = mapChar(triggered ? 9 : 0);             // Center Right -> 9 if triggered, 0 if not.
    displayData.data[SEG_LEFT]      = mapChar(matches);                       // Centre Left -> FSM Matches for current state.
    displayData.data[SEG_FAR_LEFT]  = mapChar(transitioned);                  // Far Left -> FSM has transitioned.
}

void displayFunctionADCValue() {
    displayData.data[SEG_FAR_RIGHT] = mapChar(ADC_VALUE & 0x0F);
    displayData.data[SEG_RIGHT]     = mapChar((ADC_VALUE >> 4) & 0x0F);
    displayData.data[SEG_LEFT]      = mapChar((ADC_VALUE >> 8) & 0x0F);
    displayData.data[SEG_FAR_LEFT]  = mapChar((ADC_VALUE >> 12) & 0x0F);
}

void displayFunctionAlarmSetFlag() {
    uint8_t flags                   = alarmSetFlags();
    displayData.data[SEG_FAR_RIGHT] = mapChar(flags & 0x01);        // Inc Press
    displayData.data[SEG_RIGHT]     = mapChar((flags >> 1) & 0x01); // Set Hold
    displayData.data[SEG_LEFT]      = mapChar((flags >> 2) & 0x01); // Overall
    displayData.data[SEG_FAR_LEFT]  = mapChar(BLANK);
}

void displayFunctionEEpromShow() {
     displayData.data[SEG_FAR_RIGHT]= mapChar(bytes[3]);
    displayData.data[SEG_RIGHT]     = mapChar(bytes[2]);
    displayData.data[SEG_LEFT]      = mapChar(bytes[1]);
    displayData.data[SEG_FAR_LEFT]  = mapChar(bytes[0]);
}


void displayFunctionClockDataStructData() {
     displayData.data[SEG_FAR_RIGHT]= mapChar(cd.clockData.timeMode24Hour);
    displayData.data[SEG_RIGHT]     = mapChar(cd.clockData.alarmEnabled);
    displayData.data[SEG_LEFT]      = mapChar(cd.clockData.systemState);
    displayData.data[SEG_FAR_LEFT]  = mapChar(stateMachinePtr->currentState);
}


extern uint16_t readChecksum;
extern uint16_t writeChecksum;
extern uint16_t recalChecksum;
void displayFunctionClockDataStructChecksum() {
    displayData.data[SEG_FAR_RIGHT] = mapChar(recalChecksum & 0xf);
    displayData.data[SEG_RIGHT]     = mapChar((recalChecksum >> 8 )& 0xf);
    displayData.data[SEG_LEFT]      = mapChar(readChecksum & 0xf);
    displayData.data[SEG_FAR_LEFT]  = mapChar((readChecksum >> 8) & 0xf);
}