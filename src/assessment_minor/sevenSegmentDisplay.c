#include "sevenSegmentDisplay.h"

const uint8_t SEGMENT_MAP[] = {0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8,
                               0X80, 0X90,
                               /* Continuing on for A (10) to F (15) */
                               0x88, 0x83, 0xC6, 0xA1, 0x86, 0x8E,
                               /* Then blank (16), dash (17) */
                               0xFF, 0xBF,
                               /* Decimal Point (18) */
                               0x7F};

static volatile uint8_t *m_dataIoPort = 0;
static volatile uint8_t m_dataOutPin = 0;

static volatile uint8_t *m_clkIoPort = 0;
static volatile uint8_t m_shiftPin = 0;
static volatile uint8_t m_latchPin = 0;

void InitialiseSevenSegmentDisplay(volatile uint8_t *dataDdr, volatile uint8_t dataPin, volatile uint8_t *dataIoPort, volatile uint8_t *clkDdr, volatile uint8_t *clkIoPort, volatile uint8_t shiftPin, volatile uint8_t latchPin) {
    //Save config regs/ports/pins
    m_dataOutPin = dataPin;
    m_dataIoPort = dataIoPort;

    m_clkIoPort = clkIoPort;
    m_shiftPin = shiftPin;
    m_latchPin = latchPin;

    //Configure DDR registers.
    *dataDdr |= (1 << m_dataOutPin);
    *clkDdr |= (1 << m_shiftPin | 1 << m_latchPin);
}

static inline void shiftData() {
    *m_clkIoPort = (0 << m_shiftPin);
    *m_clkIoPort = (1 << m_shiftPin);
}

static inline void latchDisplay() {
    *m_clkIoPort = (0 << m_latchPin);
    *m_clkIoPort = (1 << m_latchPin);
}

static inline void shiftBit(uint8_t bit) {
    *m_dataIoPort = (*m_dataIoPort & ~(1 << m_dataOutPin)) | (bit << m_dataOutPin);
    shiftData();
}

static inline void writeByteToSegment(uint8_t byte, uint8_t segmentNumber) {
    uint16_t buffer = (byte << 8) | segmentNumber;

    for (int i = 15; i >= 0; i--) {
        shiftBit((buffer >> i) & 1);
    }
    latchDisplay();
}

void SevenSegmentUpdate(uint8_t data[4]) {
    for (int i = 0; i < 4; i++) {
        writeByteToSegment(data[i], (1 << i));
    }
    // Clear all the displays, 0x0F targets all segments.
    writeByteToSegment(0xFF, 0x0F);
}