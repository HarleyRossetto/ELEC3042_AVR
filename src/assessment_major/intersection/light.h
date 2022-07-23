#if !defined(LIGHT_H)
#define LIGHT_H
#include "../types.h"
#include "../iotypes.h"
#include "../drivers/mcp23s17/mcp23s17.h"

typedef enum { Internal, External } LightLocation;

typedef struct {
    uint8_t port; // Subtract 0x12 to obtain IODIR port address when in BANK mode. Otherwise subtract 0x09.
    uint8_t pin;
} ExternalLight;

typedef struct {
    Port port;
    uint8_t pin;
} InternalLight;

typedef struct {
    LightLocation interfaceLocation;
    ExternalLight externalInterface;
    InternalLight internalInterface;
} Light;
#endif // LIGHT_H
