#ifndef DISPLAY_H
#define DISPLAY_H

/* Segment byte maps for numbers 0 to 9 */
const uint8_t SEGMENT_MAP[] = {0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8,
                               0X80, 0X90,
                               /* Continuing on for A (10) to F (15) */
                               0x88, 0x83, 0xC6, 0xA1, 0x86, 0x8E,
                               /* Then blank (16), dash (17) */
                               0xFF, 0xBF,
                               /* Decimal Point (18) */
                               0x7F};

#define mapChar(c) SEGMENT_MAP[c]

#define DP 18
#define BLANK 16
#define DASH 17
#define ADD_DP(segment) (segment & DP)

#define ALL_SEGMENTS 0x0F

typedef union {
    uint8_t data[4];
    uint8_t D0, D1, D2, D3;
} DisplayData;

typedef uint8_t DisplaySegment;

void displaySendData(uint8_t data, DisplaySegment segmentIndex);

void displayUpdate(DisplayData displayData);

inline void displayShiftBit();
inline void displayLatch();

#endif