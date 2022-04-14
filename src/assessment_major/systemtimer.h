/**
 * @file systemtimer.h
 * @author Harley Rossetto (44618883@students.mq.edu.au)
 * @brief Utilities for keeping track of system time using milliseconds.
 * @version 0.1
 * @date 2022-04-06
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef SYSTEMTIMER_H
#define SYSTEMTIMER_H

#include "stdint.h"

extern volatile uint64_t totalMillisecondsElasped;

uint64_t millisecondsElasped();

#endif //SYSTEMTIMER_H