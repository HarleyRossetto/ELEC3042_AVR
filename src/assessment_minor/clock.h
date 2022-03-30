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

void validateTime(volatile Clock *time);
void addTime(volatile Clock *target, int8_t hours, int8_t minutes, int8_t seconds);
void addSeconds(volatile Clock *target, int8_t seconds);
void addMinutes(volatile Clock *target, int8_t minutes);
void addHours(volatile Clock *target, int8_t hours);
ClockComparison compareClocks(volatile Clock *t1, volatile Clock *t2);

#endif //CLOCK_H