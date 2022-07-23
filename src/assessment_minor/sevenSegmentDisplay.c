/**
 * @file sevenSegmentDisplay.c
 * @author Harley Rossetto (44618883@students.mq.edu.au)
 * @brief The driver for a 4 digit, 7-segment display.
 * @version 0.1
 * @date 2022-04-06
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "sevenSegmentDisplay.h"

// Character map for what can be displayed on a segment
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
 */
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

/**
 * @brief Lowers and raises the shift pin to push the next bit into the display's shift register.
 * 
 */
static inline void shiftData() {
    *m_clkIoPort = (0 << m_shiftPin);
    *m_clkIoPort = (1 << m_shiftPin);
}

/**
 * @brief Lowers and raises the latch pin to trigger an update on the entire display.
 * 
 */
static inline void latchDisplay() {
    *m_clkIoPort = (0 << m_latchPin);
    *m_clkIoPort = (1 << m_latchPin);
}

/**
 * @brief Shifts a bit into the displays shift register.
 * 
 * @param bit Bit to be shifted in.
 */
static inline void shiftBit(uint8_t bit) {
    *m_dataIoPort = (*m_dataIoPort & ~(1 << m_dataOutPin)) | (bit << m_dataOutPin);
    shiftData();
}

/**
 * @brief Writes a byte to the display, targeting the segments specified by segmentNumber.
 * 
 * @param byte Data byte to show on segmentNumber.
 * @param segmentNumber The digit to show the byte on.
 */
static inline void writeByteToSegment(uint8_t byte, uint8_t segmentNumber) {
    // Arrange the data in a 16 bit buffer to prepare for sending
    uint16_t buffer = (byte << 8) | segmentNumber;

    // Shift each bit out of the buffer onto the display.
    for (int i = 15; i >= 0; i--) {
        shiftBit((buffer >> i) & 1);
    }
    // Latch the result so it is visible.
    latchDisplay();
}

/**
 * @brief Updates the 4 digits on the display based on the contents of data[4].
 * 
 * @param data Data to send to the display.
 */
void SevenSegmentUpdate(uint8_t data[4]) {
    for (int i = 0; i < 4; i++) {
        writeByteToSegment(data[i], (1 << i));
    }
    // Clear all the displays, 0x0F targets all segments.
    writeByteToSegment(0xFF, 0x0F);
}