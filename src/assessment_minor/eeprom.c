/**
 * @file eeprom.c
 * @author Harley Rossetto (44618883@students.mq.edu.au)
 * @brief EEPROM reading and writing module. Handles low-level reading and writing as well as
 * higher-level writing of clock data abstractions.
 * @version 0.1
 * @date 2022-04-06
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "eeprom.h"
#include "avr/interrupt.h"
#include "avr/io.h"
#include "null.h"
#include "timerTask.h"

static EEPROMState eepromState;
TimerTask *timerTaskWriteNextByte;

/**
 * @brief Initialises the eeprom module. In doing so creates a timer task used to spread out the
 * bytes written over a period of time.
 *
 */
void EEPROM_Initialise() {
    // We will create a repeating task every 50 milliseconds to update the next eeprom byte,
    // this provides aple time between writes but doesn't occur quick enough to disturb the display.
    // This task is disabled and reset when all bytes have been written.
    timerTaskWriteNextByte = TimerTaskCreate(50L, &EEPROM_WriteNextByte, NULL, false, false);
}

/**
 * @brief Reads a byte from EEPROM memory at the provided address.
 * This function disables interrupts whilst it runs.
 *
 * @param addr Address to read from.
 * @param dataOut uint8_t pointer where the data will be stored at.
 */
void EEPROM_ReadByte(uint16_t addr, uint8_t *dataOut) {
    cli();

    // Ensure any writes have been completed. Don't care about the blocking nature here because
    // the device will only read EEPROM on startup and we require this data to start from a
    // known state.
    while (EECR & (1 << EEPE))
        ;

    // Load the address
    EEAR = addr;

    // Start read by writing EEPROM Read Enable bit.
    EECR |= (1 << EERE);

    *dataOut = EEDR;
    sei();
}

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
bool EEPROM_UpdateByte(uint16_t addr, uint8_t data) {

    // First we are going to read the data that is already located at addr, if it is the same
    // as the data parameter, then we will ignore.
    uint8_t existingData;
    EEPROM_ReadByte(addr, &existingData);
    if (existingData != data) {
        cli();

        // Wait for previous write to complete.. Blocking and useless.
        // If a previous write has not been completed, just return.
        if (EECR & (1 << EEPE)) {
            return false;
        }
        // Load the address
        EEAR = addr;
        // Load the data
        EEDR = data;
        // EEPROM master Write Enable.
        EECR |= (1 << EEMPE);
        // EEPROM Write Enable.
        EECR |= (1 << EEPE);

        sei();
    }
    return true;
}

/**
 * @brief Calculates a very simple checksum which is written after all the data.
 * We can use this when reloading the system to determine if the state saved was valid or not.
 * If it was not valid then we can restore the system to some other known state, rather than to
 * a potentially incomplete one represented by the read data.
 *
 * @param buffer EEPROMClockDataStructBuffer to calculate the checksum on.
 */
static void calculateSimpleChecksum(EEPROMClockDataStructBuffer *buffer) {
    uint16_t sum = 0;
    // Determine the buffer size, because internal_checksum is stored alongside the rest of the clock data
    // we need to exclude it from overall size during the calculation.
    const uint8_t buffSize = sizeof(EEPROMClockDataStructBuffer) - sizeof(uint16_t);
    for (int i = 0; i < buffSize; i++) {
        sum += buffer->buffer[i];
    }
    buffer->clockData.internal_checksum = sum;
}

/**
 * @brief Updates the EEPROMs internal state to contain the new clock data, calculates the data checksum
 * and starts the writing process.
 * 
 * @param clockData 
 */
void EEPROM_SetClockDataStruct(ClockDataStruct clockData) {
    // Disable the write timer just in case it is running, as well as resetting it.
    TimerTaskDisable(timerTaskWriteNextByte);
    TimerTaskReset(timerTaskWriteNextByte);

    // Update the states data buffer
    eepromState.cdf          = (EEPROMClockDataStructBuffer){clockData};
    // Reset written bytes back to 0
    eepromState.bytesWritten = 0;
    // Calculate the checksum for data
    calculateSimpleChecksum(&(eepromState.cdf));

    //Enable the write next byte timer task so that the writing process may begin.
    TimerTaskEnable(timerTaskWriteNextByte);
}

/**
 * @brief Reads the Clock Data from EEPROM, recalculating the checksum to provide feedback to the caller
 * so they know whether or not the data is considered valid.
 * 
 * @return EEPROMReadClockData 
 */
EEPROMReadClockData EEPROM_ReadClockData() {
    EEPROMClockDataStructBuffer dataBuffer;

    for (uint16_t i = 0; i < sizeof(dataBuffer.buffer); i++) {
        EEPROM_ReadByte(i, &dataBuffer.buffer[i]);
    }

    uint16_t eepromReadChecksum = dataBuffer.clockData.internal_checksum;
    calculateSimpleChecksum(&dataBuffer);
    return (EEPROMReadClockData){dataBuffer.clockData, dataBuffer.clockData.internal_checksum == eepromReadChecksum};
}

/**
 * @brief Used to write the next byte from the eeprom state buffer. If a write is successful then bytes written
 * is incremented so that next time we will write the next. If the write is unsuccessful then this will allow us 
 * to reattempt later in the future.
 * If we have written all bytes in the buffer then we will disable the timer task which handles writing
 * successive bytes.
 */
void EEPROM_WriteNextByte() {
    cli();

    // If the number of bytes written is less than the size of the data buffer,
    // we will attempt to update the next byte. If that is successful, increment bytes written.
    // Otherwise we will reattempt in the future.
    if (eepromState.bytesWritten < sizeof(EEPROMClockDataStructBuffer)) {
        // If the update is successful, increment bytes written. Otherwise we will leave it be
        // so that we can reattempt next time. However because there is a 1 second delay between
        // writes, we shouldn't ever have a collision here.
        if (EEPROM_UpdateByte(eepromState.bytesWritten, eepromState.cdf.buffer[eepromState.bytesWritten])) {
            eepromState.bytesWritten++;
        }
    } else {
        // All bytes have been written, so disable the write task.
        TimerTaskDisable(timerTaskWriteNextByte);
    }

    sei();
}