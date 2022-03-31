#ifndef CLOCK_H
#define CLOCK_H

#include "stdint.h"

typedef struct  {
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
} Clock;

#define MATCH 0
typedef enum { LOWER = -1, EQUAL, HIGHER } ClockComparison;

void Clock_ValidateTime(volatile Clock *time);
void Clock_AddTime(volatile Clock *target, int8_t hours, int8_t minutes, int8_t seconds);
void Clock_AddSeconds(volatile Clock *target, int8_t seconds);
void Clock_AddMinutes(volatile Clock *target, int8_t minutes);
void Clock_AddHours(volatile Clock *target, int8_t hours);
ClockComparison Clock_CompareClocks(volatile Clock *t1, volatile Clock *t2);

#endif //CLOCK_H