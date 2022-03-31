#include "eeprom_data.h"

EEPROMData EEPROM_ReadData() {
    uint16_t address = 0;
    EEPROMDataBuffer buffer;
    for (int i = 0; i < sizeof(buffer); i++) {
        EEPROM_Read(address + i, &buffer.buffer[i]);
    }
    return buffer.eepromData;
}

// static EEPROMData advancedWip() {

//     uint16_t address = 0;
//     uint8_t offset   = INDEX_OFFSET;
//     uint8_t lastId, currentId;
//     EEPROM_Read(address, &lastId);

//     EEPROM_Read(address += offset, &currentId);

//     // Whilst the id's are ascending, keep reading. We want to find where index follows a pattern
//     // like so: 1, ..., 2, ..., 3, ..., 7, ..., 8.....
//     // Once this ascending pattern is broken, out eepromData starts from address -= INDEX_OFFSET
//     while (currentId == lastId + 1) {
//         lastId = currentId;
//         EEPROM_Read(address += offset, &currentId);
//     }
//     return (EEPROMData){};
// }

void EEPROM_WriteData(EEPROMData *eepromData) {
    EEPROMDataBuffer buffer;
    buffer.eepromData = *eepromData;
    for (int i = 0; i < sizeof(buffer); i++) {
        EEPROM_Update(buffer.buffer[i], i);
    }
}