#ifndef CLOCK_H
#define CLOCK_H

#include "stdint.h"

typedef struct  {
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
} Time;

void validateTime(Time *time);
void addTime(Time *target, uint8_t hours, uint8_t minutes, uint8_t seconds);
void addSeconds(Time *target, uint8_t seconds);
void addMinutes(Time *target, uint8_t minutes);
void addHours(Time *target, uint8_t hours);


#endif //CLOCK_H