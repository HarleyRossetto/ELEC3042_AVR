/**
 * @file led.c
 * @author Harley Rossetto (44618883@students.mq.edu.au)
 * @brief Abstractions for LEDs
 * @version 0.1
 * @date 2022-04-06
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "led.h"

/**
 * @brief Creates an LED and ensures it is off.
 * 
 * @param ddr Data Direction Register the LED is on.
 * @param port Port Register the LED is on.
 * @param pin Pin the LED is on.
 * @return LED new struct LED.
 */
LED LED_Create(Port ddr, Port port, Pin pin) {
    *ddr |= (1 << pin);
    LED led = {port, pin, OFF};
    LED_Off(&led);
    return led;
}

/**
 * @brief Turns on the LED, if not null.
 * 
 * @param led LED to turn on.
 */
void LED_On(LED *led) {
    if (!led)
        return;
    led->state = ON;
    *led->port &= ~(1 << led->pin);
}

/**
 * @brief Turns off the LED, if not null.
 * 
 * @param led LED to turn off.
 */
void LED_Off(LED *led) {
    if (!led)
        return;
    led->state = OFF;
    *led->port |= (1 << led->pin);
}

/**
 * @brief Toggles the LED if not null.
 * 
 * @param led LED to turn toggle.
 */
void LED_Toggle(LED *led) {
    if (!led)
        return;
    led->state = !led->state;
    *led->port ^= (1 << led->pin);
}
