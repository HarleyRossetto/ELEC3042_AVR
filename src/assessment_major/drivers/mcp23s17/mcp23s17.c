#include "mcp23s17.h"
#include "../spi/spi.h"
#include "avr/io.h"

#define CS        B, PB2

#define MCP23S17  64
#define DEVICE_ID 0

Initialiser MCP23S17_Initialise() {
    // Initialise the port expander so that interrupt pins are mirrored and sequential operations are initially disabled.
    // If we want to use BANK = 1, set accordingly.
#if defined(BANK_MODE_ONE)
    MCP23S17_WriteRegister(IOCON, (1 << BANK) | (1 << MIRROR) | (1 << SEQOP));
#else
    MCP23S17_WriteRegister(IOCON, (1 << MIRROR) | (1 << SEQOP));
#endif
}

inline void MCP23S17_EnableInterrupt(MCP_Port port, uint8_t pin) {
    const uint8_t intEnPort = (port == PORT_A ? GPINTENA : GPINTENB);
    MCP23S17_SetBit(intEnPort, pin);
}

inline void MCP23S17_DisableInterrupt(MCP_Port port, uint8_t pin) {
    const uint8_t intEnPort = (port == PORT_A ? GPINTENA : GPINTENB);    
    MCP23S17_ClearBit(intEnPort, pin);
}

inline void MCP23S17_WriteInterruptRegister(MCP_Port port, uint8_t reg) {
    const uint8_t intEnPort = (port == PORT_A ? GPINTENA : GPINTENB);    
    MCP23S17_WriteRegister(intEnPort, reg);
}

inline inline uint8_t MCP23S17_ReadInterruptRegister(MCP_Port port) {
    const uint8_t intEnPort = (port == PORT_A ? GPINTENA : GPINTENB);    
    return MCP23S17_ReadRegister(intEnPort);
}

inline void MCP23S17_CompareInterruptAgainstDefaultValue(MCP_Port port, uint8_t pin) {
    const uint8_t intEnPort = (port == PORT_A ? INTCONA : INTCONB);
    MCP23S17_SetBit(intEnPort, pin);
}

inline void MCP23S17_CompareInterruptAgainstPreviousValue(MCP_Port port, uint8_t pin) {
    const uint8_t intEnPort = (port == PORT_A ? INTCONA : INTCONB);    
    MCP23S17_ClearBit(intEnPort, pin);
}

inline void MCP23S17_WriteInterruptComparisonRegister(MCP_Port port, uint8_t reg) {
    const uint8_t intEnPort = (port == PORT_A ? INTCONA : INTCONB);    
    MCP23S17_WriteRegister(intEnPort, reg);
}

inline uint8_t MCP23S17_ReadInterruptComparisonRegister(MCP_Port port) {
    const uint8_t intEnPort = (port == PORT_A ? INTCONA : INTCONB);    
    return MCP23S17_ReadRegister(intEnPort);
}

inline void MCP23S17_EnablePullUp(MCP_Port port, uint8_t pin) {
    const uint8_t gppuPort = (port == PORT_A ? GPPUA : GPPUB);
    MCP23S17_SetBit(gppuPort, pin);
}

inline void MCP23S17_DisablePullUp(MCP_Port port, uint8_t pin) {
    const uint8_t gppuPort = (port == PORT_A ? GPPUA : GPPUB);
    MCP23S17_SetBit(gppuPort, pin);
}

inline void MCP23S17_WritePullUpRegister(MCP_Port port, uint8_t reg) {
    const uint8_t gppuPort = (port == PORT_A ? GPPUA : GPPUB);
    MCP23S17_WriteRegister(gppuPort, reg);
}

inline uint8_t MCP23S17_ReadPullUpRegister(MCP_Port port) {
    const uint8_t gppuPort = (port == PORT_A ? GPPUA : GPPUB);
    return MCP23S17_ReadRegister(gppuPort);
}

inline uint8_t MCP23S17_ReadInterruptFlagRegister(MCP_Port port) {
    const uint8_t intfPort = (port == PORT_A ? INTFA : INTFB);
    return MCP23S17_ReadRegister(intfPort);
}

inline uint8_t MCP23S17_ReadInterruptCaptureRegister(MCP_Port port) {
    const uint8_t intCapPort = (port == PORT_A ? INTCAPA : INTCAPB);
    return MCP23S17_ReadRegister(intCapPort);
}

////////////////////////////////////////////////////////////////////////////////
///                    IO Direction Register Manipulation                     //
////////////////////////////////////////////////////////////////////////////////

inline void MCP23S17_IoDirectionSetPin(MCP_Port port, uint8_t pin) {
    const uint8_t ioDirPort = (port == PORT_A ? IODIRA : IODIRB);
    MCP23S17_SetBit(ioDirPort, pin);
}

inline void MCP23S17_IoDirectionClearPin(MCP_Port port, uint8_t pin){
    const uint8_t ioDirPort = (port == PORT_A ? IODIRA : IODIRB);
    MCP23S17_ClearBit(ioDirPort, pin);
}

inline void MCP23S17_IoDirectionAppend(MCP_Port port, uint8_t reg) {
    const uint8_t ioDirPort = (port == PORT_A ? IODIRA : IODIRB);
    MCP23S17_Append(ioDirPort, reg);
}

inline void MCP23S17_IoDirectionClear(MCP_Port port, uint8_t reg) {
    const uint8_t ioDirPort = (port == PORT_A ? IODIRA : IODIRB);
    MCP23S17_ClearSpecified(ioDirPort, reg);
}

inline void MCP23S17_IoDirectionWrite(MCP_Port port, uint8_t reg) {
    const uint8_t ioDirPort = (port == PORT_A ? IODIRA : IODIRB);
    MCP23S17_WriteRegister(ioDirPort, reg);
}

inline uint8_t MCP23S17_IoDirectionRead(MCP_Port port) {
    const uint8_t ioDirPort = (port == PORT_A ? IODIRA : IODIRB);
    return MCP23S17_ReadRegister(ioDirPort);
}

////////////////////////////////////////////////////////////////////////////////
///                         GPIO Register Manipulation                        //
////////////////////////////////////////////////////////////////////////////////

inline void MCP23S17_GpioSetPin(MCP_Port port, uint8_t pin) {
    const uint8_t gpioPort = (port == PORT_A ? GPIOA : GPIOB);
    MCP23S17_SetBit(gpioPort, pin);
}

inline void MCP23S17_GpioClearPin(MCP_Port port, uint8_t pin) {
    const uint8_t gpioPort = (port == PORT_A ? GPIOA : GPIOB);    
    MCP23S17_ClearBit(gpioPort, pin);
}

inline void MCP23S17_GpioAppend(MCP_Port port, uint8_t reg) {
    const uint8_t gpioPort = (port == PORT_A ? GPIOA : GPIOB);
    MCP23S17_Append(gpioPort, reg);
}

inline void MCP23S17_GpioClear(MCP_Port port, uint8_t reg) {
    const uint8_t gpioPort = (port == PORT_A ? GPIOA : GPIOB);
    MCP23S17_ClearSpecified(gpioPort, reg);
}

inline void MCP23S17_GpioWriteRegister(MCP_Port port, uint8_t reg) {
    const uint8_t gpioPort = (port == PORT_A ? GPIOA : GPIOB);    
    MCP23S17_WriteRegister(gpioPort, reg);
}

inline uint8_t MCP23S17_GpioReadRegister(MCP_Port port) {
    const uint8_t gpioPort = (port == PORT_A ? GPIOA : GPIOB);    
    return MCP23S17_ReadRegister(gpioPort);
}

////////////////////////////////////////////////////////////////////////////////
///                    Register Bit Manipulation Utilities                    //
////////////////////////////////////////////////////////////////////////////////

inline void MCP23S17_SetBit(uint8_t address, uint8_t bit) {
    const uint8_t existingValue = MCP23S17_ReadRegister(address);
    MCP23S17_WriteRegister(address, existingValue | (1 << bit));
}

inline void MCP23S17_ClearBit(uint8_t address, uint8_t bit) {
    const uint8_t value = MCP23S17_ReadRegister(address);
    MCP23S17_WriteRegister(address, value & ~(1 << bit));
}

inline void MCP23S17_ToggleBit(uint8_t address, uint8_t bit) {
    const uint8_t value = MCP23S17_ReadRegister(address);
    MCP23S17_WriteRegister(address, value ^ (1 << bit));
}

inline void MCP23S17_Append(uint8_t address, uint8_t reg) {
    const uint8_t existingValue = MCP23S17_ReadRegister(address);
    MCP23S17_WriteRegister(address, existingValue | reg);
}

inline void MCP23S17_ClearSpecified(uint8_t address, uint8_t reg) {
    const uint8_t value = MCP23S17_ReadRegister(address);
    MCP23S17_WriteRegister(address, value & ~reg);
}

void MCP23S17_WriteRegister(uint8_t address, uint8_t data) {
    CLEAR_BIT(CS);

    SPI_ShiftByte(MCP23S17 | DEVICE_ID | WRITE);
    SPI_ShiftByte(address);
    SPI_ShiftByte(data);

    SET_BIT(CS);
}

uint8_t MCP23S17_ReadRegister(uint8_t address) {
    CLEAR_BIT(CS);

    SPI_ShiftByte(MCP23S17 | DEVICE_ID | READ);
    SPI_ShiftByte(address);
    uint8_t result = SPI_ShiftByte(0x00);

    SET_BIT(CS);

    return result;
}
