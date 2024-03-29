#if !defined(MCP23S17_H)
#define MCP23S17_H

#include "stdint.h"
#include "../../types.h"

#if defined(BANK_MODE_ONE)
#define IODIRA   0x00
#define IPOLA    0x01
#define GPINTENA 0x02
#define DEFVALA  0x03
#define INTCONA  0x04
#define IOCON    0x05
#define GPPUA    0x06
#define INTFA    0x07
#define INTCAPA  0x08
#define GPIOA    0x09
#define OLATA    0x0A
#define IODIRB   0x10
#define IPOLB    0x11
#define GPINTENB 0x12
#define DEFVALB  0x13
#define INTCONB  0x14
#define GPPUB    0x16
#define INTFB    0x17
#define INTCAPB  0x18
#define GPIOB    0x19
#define OLATB    0x1A
#else
#define IODIRA   0x00
#define IODIRB   0x01
#define IPOLA    0x02
#define IPOLB    0x03
#define GPINTENA 0x04
#define GPINTENB 0x05
#define DEFVALA  0x06
#define DEFVALB  0x07
#define INTCONA  0x08
#define INTCONB  0x09
#define IOCON    0x0A
#define GPPUA    0x0C
#define GPPUB    0x0D
#define INTFA    0x0E
#define INTFB    0x0F
#define INTCAPA  0x10
#define INTCAPB  0x11
#define GPIOA    0x12
#define GPIOB    0x13
#define OLATA    0x14
#define OLATB    0x15
#endif

#define READ      1
#define WRITE     0

// IOCON - I/O Expander Configuration Register
#define BANK 7
#define MIRROR 6
#define SEQOP 5
#define DISSLW 4
#define HAEN 3
#define ODR 2
#define INTPOL 1

// IODIR
#define IO7 7
#define IO6 6
#define IO5 5
#define IO4 4
#define IO3 3
#define IO2 2
#define IO1 1
#define IO0 0

// IPOL
#define IP7 7
#define IP6 6 
#define IP5 5 
#define IP4 4 
#define IP3 3
#define IP2 2
#define IP1 1
#define IP0 0

// GPINTEN
#define GPINT7 7
#define GPINT6 6
#define GPINT5 5
#define GPINT4 4
#define GPINT3 3
#define GPINT2 2
#define GPINT1 1
#define GPINT0 0

// GPPU
#define PU7 7
#define PU6 6
#define PU5 5
#define PU4 4
#define PU3 3
#define PU2 2
#define PU1 1
#define PU0 0

// GPIO
#define GP7 7
#define GP6 6
#define GP5 5
#define GP4 4
#define GP3 3
#define GP2 2
#define GP1 1
#define GP0 0

// OL
#define OL7 7
#define OL6 6
#define OL5 5
#define OL4 4
#define OL3 3
#define OL2 2
#define OL1 1
#define OL0 0

// INTCP
#define ICP7 7
#define ICP6 6
#define ICP5 5
#define ICP4 4
#define ICP3 3
#define ICP2 2
#define ICP1 1
#define ICP0 0

typedef enum { MCP_PORT_A, MCP_PORT_B } MCP_Port;

#define MCP23S17_InterruptEnable_PortA(pin) MCP23S17_InterruptEnable(MCP_PORT_A, pin);

Initialiser MCP23S17_Initialise();

////////////////////////////////////////////////////////////////////////////////
///                  Interrupt Enable Register Manipulation                   //
////////////////////////////////////////////////////////////////////////////////
void MCP23S17_InterruptEnable(MCP_Port port, uint8_t pin);
void MCP23S17_InterruptDisable(MCP_Port port, uint8_t pin);
void MCP23S17_InterruptEnableSetRegister(MCP_Port port, uint8_t reg);
void MCP23S17_PullUpAppendRegister(MCP_Port port, uint8_t reg);
void MCP23S17_PullUpClearRegister(MCP_Port port, uint8_t reg);
uint8_t MCP23S17_InterruptReadRegister(MCP_Port port);

////////////////////////////////////////////////////////////////////////////////
///                  Interrupt-On-Change Register Manipulation                //
////////////////////////////////////////////////////////////////////////////////
void MCP23S17_InterruptOnChangeSetBit(MCP_Port port, uint8_t pin);
void MCP23S17_InterruptOnChangeClearBit(MCP_Port port, uint8_t pin);
void MCP23S17_InterruptOnChangeSetRegister(MCP_Port port, uint8_t reg);
void MCP23S17_PullUpAppendRegister(MCP_Port port, uint8_t reg);
void MCP23S17_PullUpClearRegister(MCP_Port port, uint8_t reg);
uint8_t MCP23S17_InterruptOnChangeReadRegister(MCP_Port port);

////////////////////////////////////////////////////////////////////////////////
///                  Pull-Up Resistor Register Manipulation                   //
////////////////////////////////////////////////////////////////////////////////
void MCP23S17_PullUpSetBit(MCP_Port port, uint8_t pin);
void MCP23S17_PullUpClearBit(MCP_Port port, uint8_t pin);
void MCP23S17_PullUpSetRegister(MCP_Port port, uint8_t reg);
void MCP23S17_PullUpAppendRegister(MCP_Port port, uint8_t reg);
void MCP23S17_PullUpClearRegister(MCP_Port port, uint8_t reg);
uint8_t MCP23S17_PullUpReadRegister(MCP_Port port);

////////////////////////////////////////////////////////////////////////////////
///                     Interrupt Flag Register Read                          //
////////////////////////////////////////////////////////////////////////////////
uint8_t MCP23S17_InterruptFlagReadRegister(MCP_Port port);

////////////////////////////////////////////////////////////////////////////////
///                    Interrupt Flag Capture Register                        //
////////////////////////////////////////////////////////////////////////////////
uint8_t MCP23S17_InterruptCaptureReadRegister(MCP_Port port);

////////////////////////////////////////////////////////////////////////////////
///                    IO Direction Register Manipulation                     //
////////////////////////////////////////////////////////////////////////////////
void MCP23S17_IoDirectionSetPin(MCP_Port port, uint8_t pin);
void MCP23S17_IoDirectionClearPin(MCP_Port port, uint8_t pin);
void MCP23S17_IoDirectionAppendRegister(MCP_Port port, uint8_t reg);
void MCP23S17_IoDirectionClearRegister(MCP_Port port, uint8_t reg);
void MCP23S17_IoDirectionSetRegister(MCP_Port port, uint8_t reg);
uint8_t MCP23S17_IoDirectionReadRegister(MCP_Port port);

////////////////////////////////////////////////////////////////////////////////
///                         GPIO Register Manipulation                        //
////////////////////////////////////////////////////////////////////////////////
void MCP23S17_GpioSetPin(MCP_Port port, uint8_t pin);
void MCP23S17_GpioClearPin(MCP_Port port, uint8_t pin);
void MCP23S17_GpioTogglePin(MCP_Port port, uint8_t pin);
void MCP23S17_GpioAppendRegister(MCP_Port port, uint8_t pin);
void MCP23S17_GpioClearRegister(MCP_Port port, uint8_t reg);
void MCP23S17_GpioSetRegister(MCP_Port port, uint8_t reg);
uint8_t MCP23S17_GpioReadRegister(MCP_Port port);

////////////////////////////////////////////////////////////////////////////////
///                    Register Bit Manipulation Utilities                    //
////////////////////////////////////////////////////////////////////////////////
void MCP23S17_SetBit(uint8_t address, uint8_t bit);
void MCP23S17_ClearBit(uint8_t address, uint8_t bit);
void MCP23S17_ToggleBit(uint8_t address, uint8_t bit);
void MCP23S17_Append(uint8_t address, uint8_t reg);
void MCP23S17_ClearSpecified(uint8_t address, uint8_t reg);

////////////////////////////////////////////////////////////////////////////////
///                         Register Reading.Writing                          //
////////////////////////////////////////////////////////////////////////////////
void MCP23S17_SetRegister(uint8_t address, uint8_t data);
uint8_t MCP23S17_ReadRegister(uint8_t address);

#endif // MCP23S17_H
