/**
 * @file iotypes.h
 * @author Harley Rossetto (44618883@students.mq.edu.au)
 * @brief Utility macros for referencing Ports and Pins.
 * @version 0.1
 * @date 2022-04-06
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef IOTYPES_H
#define IOTYPES_H
#include "stdint.h"

typedef volatile uint8_t *Port;
typedef volatile Port DDR;
typedef Port InputRegister;
typedef uint8_t Pin;

#endif //IOTYPES_H