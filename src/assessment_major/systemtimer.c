/**
 * @file systemtimer.c
 * @author Harley Rossetto (44618883@students.mq.edu.au)
 * @brief Utilities for keeping track of system time using milliseconds.
 * @version 0.1
 * @date 2022-04-06
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "systemtimer.h"

volatile uint64_t totalMillisecondsElasped = 0;

inline uint64_t millisecondsElasped()
{
    return totalMillisecondsElasped;
}
