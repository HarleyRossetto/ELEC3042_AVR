#ifndef EEPROM_DATA_H
#define EEPROM_DATA_H

#include "stdint.h"
#include "eeprom_alarm.h"
#include "bool.h"
#include "timemode.h"
#include "fsm.h"

#define INDEX_OFFSET 5

// typedef struct 
// {
//     uint8_t INDEX : 8;
//     uint8_t timeHour : 5;
//     uint8_t timeMinute : 6;
//     uint8_t timeSecond : 6;
//     uint8_t alarmHour : 5;
//     uint8_t alarmMinute : 6;
//     bool alarmEnable : 1;
//     uint8_t currentState : 1;
//     uint8_t timeMode : 1;
// } EEPROMData;

typedef struct 
{
    uint8_t INDEX;
    uint8_t timeHour;
    uint8_t timeMinute;
    uint8_t timeSecond;
    uint8_t alarmHour;
    uint8_t alarmMinute;
    bool alarmEnable;
    FSM_STATE currentState;
    TimeMode timeMode;
} EEPROMData;

typedef union {
    EEPROMData eepromData;
    uint8_t buffer[sizeof(EEPROMData)];
} EEPROMDataBuffer;

EEPROMData EEPROM_ReadData();
void EEPROM_WriteData(EEPROMData *data);

#endif // EEPROM_DATA_H