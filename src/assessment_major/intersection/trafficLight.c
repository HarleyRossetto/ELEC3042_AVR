#include "trafficLight.h"
#include "../drivers/mcp23s17/mcp23s17.h"

static void TrafficLight_InternalClear(InternalLight *light) {
    if (!light)
        return;

    *light->port &= ~(1 << light->pin);
}

static void TrafficLight_InternalSet(InternalLight *light) {
    if (!light)
        return;

    *light->port |= (1 << light->pin);
}

static void TrafficLight_InternalToggle(InternalLight *light) {
    if (!light)
        return;

    *light->port ^= (1 << light->pin);
}

static void TrafficLight_ExternalClear(ExternalLight *light) {
    if (!light)
        return;

    MCP23S17_GpioClearPin(light->port, light->pin);
}

static void TrafficLight_ExternalSet(ExternalLight *light) {
    if (!light)
        return;

    MCP23S17_GpioSetPin(light->port, light->pin);
}

static void TrafficLight_ExternalToggle(ExternalLight *light) {
    if (!light)
        return;

    MCP23S17_GpioTogglePin(light->port, light->pin);
}

void TrafficLight_Clear(Light *light) {
    if (!light)
        return;

    if (light->interfaceLocation == Internal) {
        TrafficLight_InternalClear(&light->internalInterface);
    } else if (light->interfaceLocation == External) {
        TrafficLight_ExternalClear(&light->externalInterface);
    }
}

void TrafficLight_Set(Light *light) {
    if (!light)
        return;

    if (light->interfaceLocation == Internal) {
        TrafficLight_InternalSet(&light->internalInterface);
    } else if (light->interfaceLocation == External) {
        TrafficLight_ExternalSet(&light->externalInterface);
    }
}

void TrafficLight_Toggle(Light *light) {
    if (!light)
        return;

    if (light->interfaceLocation == Internal) {
        TrafficLight_InternalToggle(&light->internalInterface);
    } else if (light->interfaceLocation == External) {
        TrafficLight_ExternalToggle(&light->externalInterface);
    }
}

void TrafficLight_Green(TrafficLight *light) {
    if (!light)
        return;

    TrafficLight_Clear(&light->red);
    TrafficLight_Clear(&light->yellow);
    TrafficLight_Set(&light->green);
}

void TrafficLight_Yellow(TrafficLight *light) {
    TrafficLight_Clear(&light->red);
    TrafficLight_Set(&light->yellow);
    TrafficLight_Clear(&light->green);
}

void TrafficLight_Red(TrafficLight *light) {
    TrafficLight_Set(&light->red);
    TrafficLight_Clear(&light->yellow);
    TrafficLight_Clear(&light->green);
}

void TrafficLight_Blank(TrafficLight *light) {
    TrafficLight_Clear(&light->red);
    TrafficLight_Clear(&light->yellow);
    TrafficLight_Clear(&light->green);
}

void TrafficLight_Hazard(TrafficLight *light) {
    if (!light)
        return;

    TrafficLight_Toggle(&light->yellow);
}