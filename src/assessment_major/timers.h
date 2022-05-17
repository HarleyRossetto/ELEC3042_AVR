/**
 * @file timers.h
 * @author Harley Rossetto (44618883@students.mq.edu.au)
 * @brief Macros to help configure the ATmega328p's TCC modules.
 * @version 0.1
 * @date 2022-04-06
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef TIMERS_H
#define TIMERS_H

#define TC0                         TCCR0B
#define TC1                         TCCR1B
#define TC2                         TCCR2B
#define CLOCK_SELECT_NO_SOURCE      (0 << CS12) | (0 << CS11) | (0 << CS10)
#define CLOCK_SELECT_1_PRESCALER    (0 << CS12) | (0 << CS11) | (1 << CS10)
#define CLOCK_SELECT_8_PRESCALER    (0 << CS12) | (1 << CS11) | (0 << CS10)
#define CLOCK_SELECT_64_PRESCALER   (0 << CS12) | (1 << CS11) | (1 << CS10)
#define CLOCK_SELECT_256_PRESCALER  (1 << CS12) | (0 << CS11) | (0 << CS10)
#define CLOCK_SELECT_1024_PRESCALER (1 << CS12) | (0 << CS11) | (1 << CS10)

#define TIMER2_CLOCK_SELECT_64_PRESCALER ((1 << CS22) | (0 << CS21) | (0 << CS20))

#define TIMER1_CLOCK_SELECT_8_PRESCALER ((0 << CS12) | (1 << CS11) | (0 << CS10))
#define TIMER1_CLOCK_SELECT_64_PRESCALER ((1 << CS12) | (1 << CS11) | (1 << CS10))

// Waveform Generation Mode - PWM, Phase Correct Mode 11
#define TC1_TCCR1A_CFG ((1 << WGM11) | (1 << WGM10) | (1 << COM1A1))
#define TC1_WGM_MODE_15_TCCR1A ((1 << WGM11) | (1 << WGM10))
#define TC1_WGM_MODE_15_TCCR1B ((1 << WGM13) | (1 << WGM12))

#define enableTimer(timer, clockSelect) (timer = (timer & 0xF8) | clockSelect)
#define disableTimer(timer)             (timer &= 0xF8)
#define timerEnabled(timer)             (timer & 0x7)

#endif