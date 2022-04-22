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
    MCP23S17_SetRegister(IOCON, (1 << MIRROR) | (1 << SEQOP));
#endif
}

////////////////////////////////////////////////////////////////////////////////
///                  Interrupt Enable Register Manipulation                   //
////////////////////////////////////////////////////////////////////////////////
inline void MCP23S17_InterruptEnable(MCP_Port port, uint8_t pin) {
    const uint8_t intEnPort = (port == PORT_A ? GPINTENA : GPINTENB);
    MCP23S17_SetBit(intEnPort, pin);
}

inline void MCP23S17_InterruptDisable(MCP_Port port, uint8_t pin) {
    const uint8_t intEnPort = (port == PORT_A ? GPINTENA : GPINTENB);
    MCP23S17_ClearBit(intEnPort, pin);
}

inline void MCP23S17_PullUpAppendRegister(MCP_Port port, uint8_t reg) {
    const uint8_t gpintenPort = (port == PORT_A ? GPINTENA : GPINTENB);
    MCP23S17_Append(gpintenPort, reg);
}

inline void MCP23S17_PullUpClearRegister(MCP_Port port, uint8_t reg) {
    const uint8_t gpintenPort = (port == PORT_A ? GPINTENA : GPINTENB);
    MCP23S17_ClearSpecified(gpintenPort, reg);
}

inline void MCP23S17_InterruptEnableSetRegister(MCP_Port port, uint8_t reg) {
    const uint8_t intEnPort = (port == PORT_A ? GPINTENA : GPINTENB);
    MCP23S17_SetRegister(intEnPort, reg);
}

inline inline uint8_t MCP23S17_InterruptReadRegister(MCP_Port port) {
    const uint8_t intEnPort = (port == PORT_A ? GPINTENA : GPINTENB);
    return MCP23S17_ReadRegister(intEnPort);
}

////////////////////////////////////////////////////////////////////////////////
///                  Interrupt-On-Change Register Manipulation                //
////////////////////////////////////////////////////////////////////////////////
inline void MCP23S17_InterruptOnChangeSetBit(MCP_Port port, uint8_t pin) {
    const uint8_t intEnPort = (port == PORT_A ? INTCONA : INTCONB);
    MCP23S17_SetBit(intEnPort, pin);
}

inline void MCP23S17_InterruptOnChangeClearBit(MCP_Port port, uint8_t pin) {
    const uint8_t intEnPort = (port == PORT_A ? INTCONA : INTCONB);
    MCP23S17_ClearBit(intEnPort, pin);
}

inline void MCP23S17_InterruptOnChangeSetRegister(MCP_Port port, uint8_t reg) {
    const uint8_t intEnPort = (port == PORT_A ? INTCONA : INTCONB);
    MCP23S17_SetRegister(intEnPort, reg);
}

inline void MCP23S17_InterruptOnChangeAppendRegister(MCP_Port port, uint8_t reg) {
    const uint8_t intconPort = (port == PORT_A ? INTCONA : INTCONB);
    MCP23S17_Append(intconPort, reg);
}

inline void MCP23S17_InterruptOnChangeClearRegister(MCP_Port port, uint8_t reg) {
    const uint8_t intconPort = (port == PORT_A ? INTCONA : INTCONB);
    MCP23S17_ClearSpecified(intconPort, reg);
}

inline uint8_t MCP23S17_InterruptOnChangeReadRegister(MCP_Port port) {
    const uint8_t intEnPort = (port == PORT_A ? INTCONA : INTCONB);
    return MCP23S17_ReadRegister(intEnPort);
}

////////////////////////////////////////////////////////////////////////////////
///                  Pull-Up Resistor Register Manipulation                   //
////////////////////////////////////////////////////////////////////////////////
inline void MCP23S17_PullUpSetBit(MCP_Port port, uint8_t pin) {
    const uint8_t gppuPort = (port == PORT_A ? GPPUA : GPPUB);
    MCP23S17_SetBit(gppuPort, pin);
}

inline void MCP23S17_PullUpClearBit(MCP_Port port, uint8_t pin) {
    const uint8_t gppuPort = (port == PORT_A ? GPPUA : GPPUB);
    MCP23S17_SetBit(gppuPort, pin);
}

inline void MCP23S17_PullUpSetRegister(MCP_Port port, uint8_t reg) {
    const uint8_t gppuPort = (port == PORT_A ? GPPUA : GPPUB);
    MCP23S17_SetRegister(gppuPort, reg);
}

inline void MCP23S17_PullUpAppendRegister(MCP_Port port, uint8_t reg) {
    const uint8_t gppuPort = (port == PORT_A ? GPPUA : GPPUB);
    MCP23S17_Append(gppuPort, reg);
}

inline void MCP23S17_PullUpClearRegister(MCP_Port port, uint8_t reg) {
    const uint8_t gppuPort = (port == PORT_A ? GPPUA : GPPUB);
    MCP23S17_ClearSpecified(gppuPort, reg);
}

inline uint8_t MCP23S17_PullUpReadRegister(MCP_Port port) {
    const uint8_t gppuPort = (port == PORT_A ? GPPUA : GPPUB);
    return MCP23S17_ReadRegister(gppuPort);
}

////////////////////////////////////////////////////////////////////////////////
///                     Interrupt Flag Register Read                          //
////////////////////////////////////////////////////////////////////////////////
inline uint8_t MCP23S17_InterruptFlagReadRegister(MCP_Port port) {
    const uint8_t intfPort = (port == PORT_A ? INTFA : INTFB);
    return MCP23S17_ReadRegister(intfPort);
}

////////////////////////////////////////////////////////////////////////////////
///                    Interrupt Flag Capture Register                        //
////////////////////////////////////////////////////////////////////////////////
inline uint8_t MCP23S17_InterruptCaptureReadRegister(MCP_Port port) {
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

inline void MCP23S17_IoDirectionClearPin(MCP_Port port, uint8_t pin) {
    const uint8_t ioDirPort = (port == PORT_A ? IODIRA : IODIRB);
    MCP23S17_ClearBit(ioDirPort, pin);
}

inline void MCP23S17_IoDirectionAppendRegister(MCP_Port port, uint8_t reg) {
    const uint8_t ioDirPort = (port == PORT_A ? IODIRA : IODIRB);
    MCP23S17_Append(ioDirPort, reg);
}

inline void MCP23S17_IoDirectionClearRegister(MCP_Port port, uint8_t reg) {
    const uint8_t ioDirPort = (port == PORT_A ? IODIRA : IODIRB);
    MCP23S17_ClearSpecified(ioDirPort, reg);
}

inline void MCP23S17_IoDirectionWriteRegister(MCP_Port port, uint8_t reg) {
    const uint8_t ioDirPort = (port == PORT_A ? IODIRA : IODIRB);
    MCP23S17_SetRegister(ioDirPort, reg);
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

inline void MCP23S17_GpioAppendRegister(MCP_Port port, uint8_t reg) {
    const uint8_t gpioPort = (port == PORT_A ? GPIOA : GPIOB);
    MCP23S17_Append(gpioPort, reg);
}

inline void MCP23S17_GpioClearRegister(MCP_Port port, uint8_t reg) {
    const uint8_t gpioPort = (port == PORT_A ? GPIOA : GPIOB);
    MCP23S17_ClearSpecified(gpioPort, reg);
}

inline void MCP23S17_GpioWriteRegister(MCP_Port port, uint8_t reg) {
    const uint8_t gpioPort = (port == PORT_A ? GPIOA : GPIOB);
    MCP23S17_SetRegister(gpioPort, reg);
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
    MCP23S17_SetRegister(address, existingValue | (1 << bit));
}

inline void MCP23S17_ClearBit(uint8_t address, uint8_t bit) {
    const uint8_t value = MCP23S17_ReadRegister(address);
    MCP23S17_SetRegister(address, value & ~(1 << bit));
}

inline void MCP23S17_ToggleBit(uint8_t address, uint8_t bit) {
    const uint8_t value = MCP23S17_ReadRegister(address);
    MCP23S17_SetRegister(address, value ^ (1 << bit));
}

inline void MCP23S17_Append(uint8_t address, uint8_t reg) {
    const uint8_t existingValue = MCP23S17_ReadRegister(address);
    MCP23S17_SetRegister(address, existingValue | reg);
}

inline void MCP23S17_ClearSpecified(uint8_t address, uint8_t reg) {
    const uint8_t value = MCP23S17_ReadRegister(address);
    MCP23S17_SetRegister(address, value & ~reg);
}

////////////////////////////////////////////////////////////////////////////////
///                         Register Reading.Writing                          //
////////////////////////////////////////////////////////////////////////////////

void MCP23S17_SetRegister(uint8_t address, uint8_t data) {
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
