#include "eeprom_alarm.h"

void EEPROM_Read(uint16_t address, uint8_t *dataOut) {
    // If interrupts are enabled, clear them during this block.
    bool interruptsEnabled = SREG & (1 << 7);
    if (interruptsEnabled)
        cli();

    // Load the address
    EEAR = address;
    // Start read by writing EEPROM Read Enable bit.
    EECR |= (1 << EERE);
    *dataOut = EEDR;

    // If interrupts were enabled at the beginning of the block, reenable them.
    if (interruptsEnabled)
        sei();
}

void EEPROM_Update(uint8_t data, uint16_t address) {
    // If interrupts are enabled, clear them during this block.
    bool interruptsEnabled = SREG & (1 << 7);
    if (interruptsEnabled)
        cli();

    // First read the value stored at the address, if they are not equal, write the new value.
    uint8_t existingData;
    EEPROM_Read(address, &existingData);
    if (data != existingData) {

        // Wait for previous write to complete.. Blocking and useless.
        while (EECR & (1 << EEPE)) {
            ;
        }
        // Load the address
        EEAR = address;
        // Load the data
        EEDR = data;
        // EEPROM master Write Enable.
        EECR |= (1 << EEMPE);
        // EEPROM Write Enable.
        EECR |= (1 << EEPE);
    }

    // If interrupts were enabled at the beginning of the block, reenable them.
    if (interruptsEnabled)
        sei();
}