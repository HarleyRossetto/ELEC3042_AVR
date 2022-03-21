#include "clock.h"

void validateTime(volatile Time *time) {
    if (!time)
        return;

    if (time->seconds == 60)
    {
        time->minutes++;
        time->seconds = 0;
    }
    if (time->minutes == 60) {
        time->hours++;
        time->minutes = 0;
    }
    if (time->hours == 24) {
        time->hours = 0;
        time->minutes = 0;
        time->seconds = 0;
    }
}

void addTime(volatile Time *target, uint8_t hours, uint8_t minutes, uint8_t seconds) {
    if (!target)
        return;

    target->seconds += seconds;
    target->minutes += minutes;
    target->hours += hours;

    validateTime(target);
}

void addSeconds(volatile Time *target, uint8_t seconds) {
    addTime(target, 0, 0, seconds);
}

void addMinutes(volatile Time *target, uint8_t minutes) {
    addTime(target, 0, minutes, 0);
}

void addHours(volatile Time *target, uint8_t hours) {
    addTime(target, hours, 0, 0);
}