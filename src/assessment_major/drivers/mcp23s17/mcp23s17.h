#if !defined(MCP23S17_H)
#define MCP23S17_H

#include "stdint.h"
#include "../../types.h"

#if defined(BANK_MODE_ONE)
#define IODRA    0x00
#define IPOLA    0x01
#define GPINTENA 0x02
#define DEFVALA  0x03
#define ITCONA   0x04
#define IOCON    0x05
#define GPPUA    0x06
#define INTFA    0x07
#define INTCAPA  0x08
#define GPIOA    0x09
#define OLATA    0x0A
#define IODRB    0x10
#define IPOLB    0x11
#define GPINTENB 0x12
#define DEFVALB  0x13
#define ITCONB   0x14
#define GPPUB    0x16
#define INTFB    0x17
#define INTCAPB  0x18
#define GPIOB    0x19
#define OLATB    0x1A
#else
#define IODRA    0x00
#define IODRB    0x01
#define IPOLA    0x02
#define IPOLB    0x03
#define GPINTENA 0x04
#define GPINTENB 0x05
#define DEFVALA  0x06
#define DEFVALB  0x07
#define ITCONA   0x08
#define ITCONB   0x09
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
#define MIRROR 7
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


Initialiser MCP23S17_Initialise();

void MCP23S17_WriteRegister(uint8_t address, uint8_t data);
uint8_t MCP23S17_ReadRegister(uint8_t address);

#endif // MCP23S17_H
