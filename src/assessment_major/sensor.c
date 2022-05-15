#include "sensor.h"

#include "systemtimer.h"

// Determines if sensor is pressed or not.
void Sensor_CheckState(uint8_t flags, uint8_t changeBits, uint8_t pin, Sensor *sensor) {
    if (!sensor)
        return;

    if (totalMillisecondsElasped - sensor->lastTime < SENSOR_DEBOUNCE) {
        return;
    }

    sensor->lastTime = totalMillisecondsElasped;
    if (flags & (1 << pin)) {
        if (changeBits & (1 << pin)) // Logic level high if left unpressed.
            sensor->state = RELEASED;
        else {
            sensor->state = PRESSED;
            sensor->triggered = PRESSED;
        }
    }
}

void Sensor_ClearTriggeredFlag(Sensor *sensor) {
    if (!sensor)
        return;

    sensor->triggered = RELEASED;
}
