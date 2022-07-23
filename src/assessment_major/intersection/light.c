#include "light.h"

ExternalLight Light_CreateExternal(MCP_Port mcpPort, uint8_t pin) { 
    return (ExternalLight){mcpPort, pin};
}

InternalLight Light_CreateInternal(Port port, uint8_t pin)
{
    *port |= (1 << pin);
    return (InternalLight){port, pin};
}