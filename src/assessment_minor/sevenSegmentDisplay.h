/**
 * @file sevenSegmentDisplay.h
 * @author Harley Rossetto (44618883@students.mq.edu.au)
 * @brief The driver for a 4 digit, 7-segment display.
 * @version 0.1
 * @date 2022-04-06
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef SEVENSEGMENTDISPLAY_H
#define SEVENSEGMENTDISPLAY_H

#include "stdint.h"

extern const uint8_t SEGMENT_MAP[];


#define mapChar(c) SEGMENT_MAP[c]

// Macros which refer to the index in SEGMENT_MAP[] where these special characters occur.
#define DP 18
#define BLANK 16
#define DASH 17

#define ALL_SEGMENTS 0x0F

#define SEG_FAR_LEFT 0
#define SEG_LEFT 1
#define SEG_RIGHT 2
#define SEG_FAR_RIGHT 3

typedef union {
    uint8_t data[4];
    uint8_t D0, D1, D2, D3;
} DisplayData;

/**
 * @brief Initialises the 7-segment display unit.
 * 
 * @param dataDdr Data Direction Register the data pin is on.
 * @param dataPin The data signal is on.
 * @param dataIoPort The Port the data pin is on.
 * @param clkDdr  Data Direction Register the clock signals are on.
 * @param clkIoPort The Port the clock signals are on.
 * @param shiftPin The shift pin.
 * @param latchPin The latch pin.
 */void InitialiseSevenSegmentDisplay(volatile uint8_t *dataDdr, uint8_t dataPin, volatile uint8_t *dataIoPort, volatile uint8_t *clkDdr, volatile uint8_t *clkIoPort, uint8_t shiftPin, uint8_t latchPin);

/**
 * @brief Updates the 4 digits on the display based on the contents of data[4].
 * 
 * @param data Data to send to the display.
 */
void SevenSegmentUpdate(uint8_t data[4]);

#endif //SEVENSEGMENTDISPLAY_H