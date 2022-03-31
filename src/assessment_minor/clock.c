#include "clock.h"

/**
 *  @brief  Validates the time struct ensuring seconds, minutes and hours overflow correctly when
 *          both incrementing and decrementing.
 * @param time A pointer to the Clock type to validate, if null then function returns.
 */
void Clock_ValidateTime(volatile Clock *time) {
    if (!time)
        return;

    // Add minute, set seconds to 0
    if (time->seconds == 60) {          
        time->minutes++;
        time->seconds = 0;
    } 
    // Because we are using unsigned values for s/m/h when 'decrementing' the value should overflow (i.e. = 255)
    // Thus if this happens then flip value back to its maximum before an overflow occurs.
    else if (time->seconds > 60) {   
        time->seconds = 59;
    }
    // Add hour, set minutes to 0.
    if (time->minutes == 60) {
        time->hours++;
        time->minutes = 0;
    } 
    // Because we are using unsigned values for s/m/h when 'decrementing' the value should overflow (i.e. = 255)
    // Thus if this happens then flip value back to its maximum before an overflow occurs.
    else if (time->minutes > 60) {
        time->minutes = 59;
    }
    // New day, set all to 0.
    if (time->hours == 24) {
        time->hours = 0;
        time->minutes = 0;
        time->seconds = 0;
    } 
    // Because we are using unsigned values for s/m/h when 'decrementing' the value should overflow (i.e. = 255)
    // Thus if this happens then flip value back to its maximum before an overflow occurs.
    else if (time->hours > 24) {
        time->hours = 23;
    }
}

/**
 * @brief Adds hours, minutes and seconds to the target time and validates the result. Handles overflows between all inputs,
 *        and existing values.
 * 
 * @param target A pointer to the target Clock type to add the time too.
 * @param hours Hours to add to the time.
 * @param minutes Minutes to add to the time.
 * @param seconds Seconds to add to the time.
 * 
 * @todo Needs to be validated itself, maths appears to be off when subtacting values.
 */
void Clock_AddTime(volatile Clock *target, int8_t hours, int8_t minutes, int8_t seconds) {
    if (!target)
        return;

    // uint16_t accumulatedSeconds = target->seconds + seconds;
    // uint16_t accumulatedMinutes = target->minutes + minutes + accumulatedSeconds / 60;
    // uint16_t accumulatedHours   = target->hours   + hours   + accumulatedMinutes / 60;

    // target->seconds = accumulatedSeconds % 60;
    // target->minutes = accumulatedMinutes % 60;
    // target->hours = accumulatedHours % 24;

    /*
        // Handle parameter overflows.
        minutes += seconds / 60;    // How many minutes represented in second value.
        seconds %= 60;              // Remainder of seconds after removal of minutes.
        hours += minutes / 60;      // How many hours represented in minute value.
        minutes %= 60;              // Remainder of minutes after removal of hours

        uint8_t accSeconds = target->seconds + seconds;                     // Target seconds + input seconds
        uint8_t accMinutes = target->minutes + minutes + accSeconds / 60;   // Target minutes + input minutes + minutes from seconds
        uint8_t accHours = target->hours + hours + accMinutes / 60;         // Target hours + input hours + hours from minutes

        target->seconds = accSeconds % 60;
        target->minutes = accMinutes % 60;
        target->hours = accHours % 24;
        */

    target->seconds += seconds;
    target->minutes += minutes;
    target->hours += hours;

    Clock_ValidateTime(target);
}

/**
 * @brief Adds the specified seconds to the target time.
 * 
 * @param target A pointer to the target Clock type to add seconds too.
 * @param seconds The number of seconds to add (or if negative value, subtract).
 */
void Clock_AddSeconds(volatile Clock *target, int8_t seconds) {
    Clock_AddTime(target, 0, 0, seconds);
}

/**
 * @brief Adds the specified minutes to the target time.
 * 
 * @param target A pointer to the target Clock type to add seconds too.
 * @param minutes The number of minutes to add (or if negative value, subtract).
 */
void Clock_AddMinutes(volatile Clock *target, int8_t minutes) {
    Clock_AddTime(target, 0, minutes, 0);
}

/**
 * @brief Adds the specified hours to the target time.
 * 
 * @param target A pointer to the target Clock type to add seconds too.
 * @param hours The number of hours to add (or if negative value, subtract).
 */
void Clock_AddHours(volatile Clock *target, int8_t hours) {
    Clock_AddTime(target, hours, 0, 0);
}

static ClockComparison compare(uint8_t v1, uint8_t v2) {
    if (v1 > v2)
        return HIGHER;
    else if (v1 < v2)
        return LOWER;
    else
        return EQUAL;
}

ClockComparison Clock_CompareClocks(volatile Clock *t1, volatile Clock *t2) {
    if (!t1 || !t2)
        return LOWER;

    ClockComparison comp = compare(t1->hours, t2->hours);
    if (comp == EQUAL) {
        comp = compare(t1->minutes, t2->minutes);
        if (comp == EQUAL) {
            return compare(t1->seconds, t2->seconds);
        }
        return comp;
    }
    return comp;
}