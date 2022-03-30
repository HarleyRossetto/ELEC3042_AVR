#include "led.h"

LED LED_Create(Port ddr, Port port, Pin pin) {
    *ddr |= (1 << pin);
    LED led = {port, pin, OFF};
    LED_Off(&led);
    return led;
}

void LED_On(LED *led) {
    if (!led)
        return;
    led->state = ON;
    *led->port &= ~(1 << led->pin);
}

void LED_Off(LED *led) {
    if (!led)
        return;
    led->state = OFF;
    *led->port |= (1 << led->pin);
}

void LED_Toggle(LED *led) {
    if (!led)
        return;
    led->state = !led->state;
    *led->port ^= (1 << led->pin);
}
