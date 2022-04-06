#ifndef EEPROM_2_H
#define EEPROM_2_H

#include "stdint.h"
#include "bool.h"

typedef struct  {
    uint8_t timeHour;
    uint8_t timeMinute;
    uint8_t alarmHour;
    uint8_t alarmMinute;
    uint8_t systemState;
    uint8_t alarmEnabled;
    uint8_t timeMode24Hour;
    uint8_t stateBeforeAlarmMode;
    uint16_t internal_checksum;
} ClockDataStruct;

typedef struct {
    ClockDataStruct clockData;
    bool isValid;
} EEPROMReadClockData;

typedef union {
    ClockDataStruct clockData;
    uint8_t buffer[sizeof(ClockDataStruct)];
} EEPROMClockDataStructBuffer;

typedef struct {
    EEPROMClockDataStructBuffer cdf;
    uint8_t bytesWritten;
} EEPROMState;

void EEPROM_SetClockDataStruct(ClockDataStruct clockData);
EEPROMReadClockData EEPROM_ReadClockData();

void EEPROM_WriteNextByte();
void EEPROM_Initialise();
void EEPROM_ReadByte(uint16_t addr, uint8_t *dataOut);
bool EEPROM_UpdateByte(uint16_t addr, uint8_t data);

#endif //EEPROM_2_H