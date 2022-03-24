#include "systemtimer.h"

volatile uint64_t totalMillisecondsElasped = 0;
volatile uint64_t millisecondsHandledBetweenTicks = 0;

inline uint64_t millisecondsElasped()
{
    return totalMillisecondsElasped;
}

inline void addMillisToSystemCounter(uint64_t millis) {
    totalMillisecondsElasped += millis;
}
