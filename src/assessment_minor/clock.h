#ifndef CLOCK_H
#define CLOCK_H

#include "stdint.h"
#include "avr/interrupt.h"

typedef struct  {
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
} Clock;

typedef enum { LOWER = -1, EQUAL, HIGHER } ClockComparison;

/**
 *  @brief  Validates the time struct ensuring seconds, minutes and hours overflow correctly when
 *          both incrementing and decrementing.
 * @param time A pointer to the Clock type to validate, if null then function returns.
 */
void Clock_ValidateTime(volatile Clock *time);

/**
 * @brief Adds hours, minutes and seconds to the target time and validates the result. Handles overflows between all inputs,
 *        and existing values.
 *
 * @param target A pointer to the target Clock type to add the time too.
 * @param hours Hours to add to the time.
 * @param minutes Minutes to add to the time.
 * @param seconds Seconds to add to the time.
 *
 */
void Clock_AddTime(volatile Clock *target, int8_t hours, int8_t minutes, int8_t seconds);

/**
 * @brief Adds the specified seconds to the target time.
 *
 * @param target A pointer to the target Clock type to add seconds too.
 * @param seconds The number of seconds to add (or if negative value, subtract).
 */
void Clock_AddSeconds(volatile Clock *target, int8_t seconds);

/**
 * @brief Adds the specified minutes to the target time.
 *
 * @param target A pointer to the target Clock type to add seconds too.
 * @param minutes The number of minutes to add (or if negative value, subtract).
 */
void Clock_AddMinutes(volatile Clock *target, int8_t minutes);

/**
 * @brief Adds the specified hours to the target time.
 *
 * @param target A pointer to the target Clock type to add seconds too.
 * @param hours The number of hours to add (or if negative value, subtract).
 */
void Clock_AddHours(volatile Clock *target, int8_t hours);

/**
 * @brief Adds hours, minutes and seconds to the target time and validates the result. Handles overflows between all inputs,
 *        and existing values.
 *
 * @param target A pointer to the target Clock type to add the time too.
 * @param hours Hours to add to the time.
 * @param minutes Minutes to add to the time.
 * @param seconds Seconds to add to the time.
 *
 */
ClockComparison Clock_CompareClocks(volatile Clock *t1, volatile Clock *t2);

#endif //CLOCK_H