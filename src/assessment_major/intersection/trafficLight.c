#include "trafficLight.h"
#include "../drivers/mcp23s17/mcp23s17.h"

void TrafficLight_SetState(TrafficLight *light, TrafficLightState state) {
    if (!light)
        return;

    switch (state) {
        case RED:
            TrafficLight_Red(light);
            break;
        case YELLOW:
            TrafficLight_Yellow(light);
            break;
        case GREEN:
            TrafficLight_Green(light);
            break;
        default:
            TrafficLight_Blank(light);
    }
}

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
    TrafficLight_Clear(&light->green);
    
    TrafficLight_Set(&light->yellow);
}

void TrafficLight_Red(TrafficLight *light) {
    TrafficLight_Clear(&light->yellow);
    TrafficLight_Clear(&light->green);

    TrafficLight_Set(&light->red);
}

void TrafficLight_Blank(TrafficLight *light) {
    TrafficLight_Clear(&light->red);
    TrafficLight_Clear(&light->yellow);
    TrafficLight_Clear(&light->green);
}

void TrafficLight_Hazard(TrafficLight *light) {
    if (!light)
        return;

    //Dont clear red if it matches yellow - these are pedestrian lights and red and yellow are the same light.
    // Going to assume if red in external, than orange is too, and vice versa for internal.
    if (light->red.interfaceLocation == External) {
        if (light->red.externalInterface.pin != light->yellow.externalInterface.pin) {
            TrafficLight_Clear(&light->red);
        }
    } else if (light->red.interfaceLocation == Internal) {
    if (light->red.internalInterface.pin != light->yellow.internalInterface.pin) {
            TrafficLight_Clear(&light->red);
        }
    }

    TrafficLight_Clear(&light->green);
    TrafficLight_Toggle(&light->yellow);
}