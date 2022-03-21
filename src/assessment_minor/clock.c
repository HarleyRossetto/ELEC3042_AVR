#include "clock.h"

void validateTime(Time *time) {
    if (time->seconds == 60) {
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

void addTime(Time *target, uint8_t hours, uint8_t minutes, uint8_t seconds) {
    target->seconds += seconds;
    target->minutes += minutes;
    target->hours += hours;

    validateTime(target);
}

void addSeconds(Time *target, uint8_t seconds) {
    addTime(target, 0, 0, seconds);
}

void addMinutes(Time *target, uint8_t minutes) {
    addTime(target, 0, minutes, 0);
}

void addHours(Time *target, uint8_t hours) {
    addTime(target, hours, 0, 0);
}