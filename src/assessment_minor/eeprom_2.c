#include "eeprom_2.h"
#include "avr/io.h"
#include "timerTask.h"
#include "null.h"
#include "avr/interrupt.h"

TimerTask *timerTaskWriteNextByte;

void EEPROM_Initialise() { 
    // We will create a repeating task every 50 milliseconds to update the next eeprom byte,
    // this provides aple time between writes but doesn't occur quick enough to disturb the display.
    // This task is disabled and reset when all bytes have been written.
    timerTaskWriteNextByte = TimerTaskCreate(50L, &EEPROM_WriteNextByte, NULL, false, false);
}

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

 void calculateSimpleChecksum(EEPROMClockDataStructBuffer *buffer) { 
    uint16_t sum = 0;
    // Determine the buffer size, because internal_checksum is stored alongside the rest of the clock data
    // we need to exclude it from overall size during the calculation.
    const uint8_t buffSize   = sizeof(EEPROMClockDataStructBuffer) - sizeof(uint16_t);
    for (int i = 0; i < buffSize; i++) {
        sum += buffer->buffer[i];
    }
    buffer->clockData.internal_checksum = sum;
 }

static EEPROMState eepromState;
uint16_t writeChecksum;
void EEPROM_SetClockDataStruct(ClockDataStruct clockData) {
    TimerTaskDisable(timerTaskWriteNextByte);
    TimerTaskReset(timerTaskWriteNextByte);

    eepromState.cdf = (EEPROMClockDataStructBuffer){clockData}; 
    eepromState.bytesWritten = 0;
    calculateSimpleChecksum(&(eepromState.cdf));
    writeChecksum = eepromState.cdf.clockData.internal_checksum;

    TimerTaskEnable(timerTaskWriteNextByte);
}

uint16_t readChecksum;
uint16_t recalChecksum;
EEPROMReadClockData EEPROM_ReadClockData() {
    EEPROMClockDataStructBuffer dataBuffer;

    for (uint16_t i = 0; i < sizeof(dataBuffer.buffer); i++) {
        EEPROM_ReadByte(i, &dataBuffer.buffer[i]);
    }

    uint16_t eepromReadChecksum = dataBuffer.clockData.internal_checksum;
    readChecksum                = eepromReadChecksum;
    calculateSimpleChecksum(&dataBuffer);
    recalChecksum = dataBuffer.clockData.internal_checksum;
    return (EEPROMReadClockData){dataBuffer.clockData, dataBuffer.clockData.internal_checksum == eepromReadChecksum};
}

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