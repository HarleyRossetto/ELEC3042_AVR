#ifndef SEVENSEGMENTDISPLAY_H
#define SEVENSEGMENTDISPLAY_H

#include "stdint.h"

extern const uint8_t SEGMENT_MAP[];

#define mapChar(c) SEGMENT_MAP[c]

#define DP 18
#define BLANK 16
#define DASH 17
#define ADD_DP(segment) (segment & DP)

#define ALL_SEGMENTS 0x0F

#define SEG_FAR_LEFT 0
#define SEG_LEFT 1
#define SEG_RIGHT 2
#define SEG_FAR_RIGHT 3

typedef union {
    uint8_t data[4];
    uint8_t D0, D1, D2, D3;
} DisplayData;

typedef uint8_t DisplaySegment;

void InitialiseSevenSegmentDisplay(volatile uint8_t *dataDdr, uint8_t dataPin, volatile uint8_t *dataIoPort, volatile uint8_t *clkDdr, volatile uint8_t *clkIoPort, uint8_t shiftPin, uint8_t latchPin);
void SevenSegmentUpdate(uint8_t data[4]);

#endif //SEVENSEGMENTDISPLAY_H