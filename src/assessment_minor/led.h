#ifndef LED_H
#define LED_H

#include "iotypes.h"

typedef enum { OFF, ON } LED_State;

typedef struct {
    Port port;
    Pin pin;
    LED_State state;
} LED;

LED LED_Create(Port ddr, Port port, Pin pin);
void LED_On(LED *led);
void LED_Off(LED *led);
void LED_Toggle(LED *led);

#endif //LED_H