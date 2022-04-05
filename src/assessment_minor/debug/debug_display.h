#ifndef DEBUG_DISPLAY_H
#define DEBUG_DISPLAY_H

#include "../sevenSegmentDisplay.h"
#include "stdint.h"
#include "../bool.h"
#include "../button.h"
#include "../null.h"
#include "../fsm.h"
#include "../eeprom_data.h"
#include "../adc.h"

extern bool transitioned;
extern uint8_t matches;
extern bool triggered;
extern FSM_TRANSITION_TABLE *stateMachinePtr;

extern DisplayData displayData;

void displayFunctionIncHold();

extern volatile uint8_t isrTime;
extern volatile uint8_t isrTimeEnd;
void displayFunctionISRTime();

void displayFunctionTwo8BitReg(volatile uint8_t *r1, volatile uint8_t *r2);

void displayFunction8BitReg(volatile uint8_t *r1);

void displayFunctionPCICR();

void displayFunctionDDRC();

void displayFunctionButtonStates();

void displayFunctionCurrentState();


void displayFunctionADCValue();

extern uint8_t alarmSetFlags();
void displayFunctionAlarmSetFlag();


void displayFunctionSizeOf();

#endif // DEBUG_DISPLAY_H