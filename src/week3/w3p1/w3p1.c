#include "avr/io.h"
#include "stdint.h"

void delay_ms(long num) {
    while (num--) {
        for (volatile long x = 0; x < 468; x++) {
            ;
        }
    }
}
void setup() {
    DDRD = 0b10010000;
    PORTD = 0b00000000;
    DDRB = 0b00000001;
    PORTB = 0b00000000;
}

#define SFTCLK PD7
#define LCHCLK PD4
#define DATAIN PB0

inline void shiftBit() {
    PORTD = (0 << SFTCLK);
    PORTD = (1 << SFTCLK);
}

inline void latchSegmentDisplay() {
    PORTD = (0 << LCHCLK);
    PORTD = (1 << LCHCLK);
}

// send the 8 bits of data in segments (top bit first)
// then the 8 bits of data in digits (top bit first)

void sendData(uint8_t segments, uint8_t digits) {
    // TODO: write the 16 bits of data using PD4 and PD7 then clock it to the output using PB0

    uint16_t buffer = (segments << 8) | digits;

    for (int i = 15; i >= 0; i--) {
        uint8_t bitToSend = (buffer >> i) & 1;

        PORTB = (PORTB & ~(1 << PB0)) | (bitToSend << DATAIN);
        shiftBit();
    }

    latchSegmentDisplay();
}

/* Segment byte maps for numbers 0 to 9 */
const uint8_t SEGMENT_MAP[] = {
    0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0X80, 0X90,
    /* Continuing on for A (10) to F (15) */
    0x88, 0x83, 0xC6, 0xA1, 0x86, 0x8E,
    /* Then blank (16), dash (17) */
    0xFF, 0xBF
};

uint8_t segmentMap(uint8_t value) {
    return SEGMENT_MAP[value];
}

long brightness[] = {0, 1, 10, 40};
uint8_t digits[4];

void showDigits() {
    for (int i = 0; i < 4; i++) {
        sendData(segmentMap(digits[i]), (1 << i));
    }
    sendData(segmentMap(16), 0);
}

int main(void) {
    setup(); // set up the physical hardware

    long loop_num = 0;
    digits[0] = 0;
    digits[1] = 6;
    digits[2] = 6;
    digits[3] = 6;

    while (1) {
        for (int i = 0; i < 100; i++) {
            long delay = (loop_num / 100) % 4;
            showDigits();
            delay_ms(brightness[delay]);
            digits[0] = delay;
        }
        loop_num++;
    }
}