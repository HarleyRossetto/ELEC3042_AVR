#if !defined(SENSOR_H)
#define SENSOR_H
#include "stdint.h"
#include "bool.h"
#include "iotypes.h"

#define SENSOR_DEBOUNCE 10

typedef struct {
    uint64_t lastTime;
    bool state;
    bool triggered;
    uint8_t periods_held;
} Sensor;

#define PRESSED 1
#define RELEASED 0

void Sensor_CheckState_External(uint8_t flags, uint8_t changeBits, uint8_t pin, Sensor *sensor);
void Sensor_CheckState_Internal(InputRegister pinReg, uint8_t pin, Sensor *sensor);
void Sensor_ClearTriggeredFlag(Sensor *sensor);

#endif // SENSOR_H
