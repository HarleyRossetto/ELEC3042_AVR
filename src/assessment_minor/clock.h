#ifndef CLOCK_H
#define CLOCK_H

#include "stdint.h"

typedef struct  {
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
} Time;

void validateTime(volatile Time *time);
void addTime(volatile Time *target, int8_t hours, int8_t minutes, int8_t seconds);
void addSeconds(volatile Time *target, int8_t seconds);
void addMinutes(volatile Time *target, int8_t minutes);
void addHours(volatile Time *target, int8_t hours);


#endif //CLOCK_H