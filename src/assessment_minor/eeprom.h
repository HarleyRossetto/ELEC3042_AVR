/**
 * @file eeprom.h
 * @author Harley Rossetto (44618883@students.mq.edu.au)
 * @brief EEPROM reading and writing module. Handles low-level reading and writing as well as
 * higher-level writing of clock data abstractions.
 * @version 0.1
 * @date 2022-04-06
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef EEPROM_H
#define EEPROM_H

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

/**
 * @brief EEPROMReadClockData is returned to consumers of this module.
 * Contains system state data inside of clockData, as well as a flag indicating
 * if the data is valid (in isValid).
 * 
 */
typedef struct {
    ClockDataStruct clockData;
    bool isValid;
} EEPROMReadClockData;

/**
 * @brief Dirty trick to make writing of data easier by having the clockData overlapped with
 * an array.
 * 
 */
typedef union {
    ClockDataStruct clockData;
    uint8_t buffer[sizeof(ClockDataStruct)];
} EEPROMClockDataStructBuffer;

/**
 * @brief State information for the EEPROM module.
 * 
 */
typedef struct {
    EEPROMClockDataStructBuffer cdf;
    uint8_t bytesWritten;
} EEPROMState;

/**
 * @brief Initialises the eeprom module. In doing so creates a timer task used to spread out the 
 * bytes written over a period of time.
 * 
 */
void EEPROM_Initialise();

/**
 * @brief Updates the EEPROMs internal state to contain the new clock data, calculates the data checksum
 * and starts the writing process.
 * 
 * @param clockData 
 */
void EEPROM_SetClockDataStruct(ClockDataStruct clockData);

/**
 * @brief Reads the Clock Data from EEPROM, recalculating the checksum to provide feedback to the caller
 * so they know whether or not the data is considered valid.
 * 
 * @return EEPROMReadClockData 
 */
EEPROMReadClockData EEPROM_ReadClockData();

/**
 * @brief Used to write the next byte from the eeprom state buffer. If a write is successful then bytes written
 * is incremented so that next time we will write the next. If the write is unsuccessful then this will allow us 
 * to reattempt later in the future.
 * If we have written all bytes in the buffer then we will disable the timer task which handles writing
 * successive bytes.
 */
void EEPROM_WriteNextByte();

/**
 * @brief Reads a byte from EEPROM memory at the provided address.
 * This function disables interrupts whilst it runs.
 * 
 * @param addr Address to read from.
 * @param dataOut uint8_t pointer where the data will be stored at.
 */
void EEPROM_ReadByte(uint16_t addr, uint8_t *dataOut);

/**
 * @brief Updates the bytes at the provided address if it is different from what is already in memory
 * in order to reduce wear on the cell.
 * 
 * @param addr Address to write to.
 * @param data Data to write.
 * @return true If the write was successful or the data matched the existing data.
 * @return false If the EEPROM module was still busy with another write. By returning false in this case
 * callees can reattempt this byte again in the future.
 */
bool EEPROM_UpdateByte(uint16_t addr, uint8_t data);

#endif //EEPROM_H