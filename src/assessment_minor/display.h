#ifndef DISPLAY_H
#define DISPLAY_H

/* Segment byte maps for numbers 0 to 9 */
const uint8_t SEGMENT_MAP[] = {
    0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0X80, 0X90,
    /* Continuing on for A (10) to F (15) */
    0x88, 0x83, 0xC6, 0xA1, 0x86, 0x8E,
    /* Then blank (16), dash (17) */
    0xFF, 0xBF,
    /* Decimal Point (18) */
    0x7F};

#define DP 18
#define BLANK 16
#define DASH 17
#define ADD_DP(segment) segment &DP

#endif