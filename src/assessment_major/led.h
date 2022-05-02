/**
 * @file led.h
 * @author Harley Rossetto (44618883@students.mq.edu.au)
 * @brief Abstractions for LEDs
 * @version 0.1
 * @date 2022-04-06
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef LED_H
#define LED_H

#include "iotypes.h"

typedef struct {
    Port port;
    Pin pin;
} LED;

/**
 * @brief Creates an LED and ensures it is off.
 * 
 * @param ddr Data Direction Register the LED is on.
 * @param port Port Register the LED is on.
 * @param pin Pin the LED is on.
 * @return LED new struct LED.
 */
LED LED_Create(Port ddr, Port port, Pin pin);

/**
 * @brief Turns on the LED, if not null.
 * 
 * @param led LED to turn on.
 */
void LED_On(LED *led);

/**
 * @brief Turns off the LED, if not null.
 * 
 * @param led LED to turn off.
 */
void LED_Off(LED *led);

/**
 * @brief Toggles the LED if not null.
 * 
 * @param led LED to turn toggle.
 */
void LED_Toggle(LED *led);

#endif //LED_H