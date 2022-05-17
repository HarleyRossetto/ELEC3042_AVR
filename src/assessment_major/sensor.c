#include "sensor.h"
#include "systemtimer.h"

// Determines if sensor is pressed or not.
void Sensor_CheckState_External(uint8_t flags, uint8_t changeBits, uint8_t pin, Sensor *sensor) {
    if (!sensor)
        return;

    const uint64_t TIME_DELTA = totalMillisecondsElasped - sensor->lastTime;

    if (TIME_DELTA < SENSOR_DEBOUNCE)
        return;

    if (flags & (1 << pin)) {
        if (changeBits & (1 << pin)) { // Logic level high if left unpressed.
            if (sensor->state == PRESSED) {
            sensor->state = RELEASED;
            }
        }
        else {
            sensor->state = PRESSED;
            sensor->triggered = PRESSED;
        }
    }

    sensor->lastTime = totalMillisecondsElasped;
}

void Sensor_CheckState_Internal(InputRegister pinReg, uint8_t pin, Sensor *sensor) {
    if (!sensor)
        return;

    const uint64_t TIME_DELTA = totalMillisecondsElasped - sensor->lastTime;
    if (TIME_DELTA < SENSOR_DEBOUNCE)
        return;


    // If released
    if (*pinReg & (1 << pin)) {
        // Sensor currently pressed
        if (sensor->state == PRESSED) {
            sensor->state = RELEASED;
        }

    } else { // Pressed
        sensor->state = PRESSED;
        sensor->triggered = PRESSED;
    }

    sensor->lastTime = totalMillisecondsElasped;
}

extern Sensor intersection_change_trigger_sensor;
void Sensor_ClearTriggeredFlag(Sensor *sensor) {
    if (!sensor)
        return;

    intersection_change_trigger_sensor = *sensor; // To access timing information.

    sensor->triggered = RELEASED;
    sensor->periods_held = 0; // Clear periods held.
}
