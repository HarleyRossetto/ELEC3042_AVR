#ifndef TIMERS_H
#define TIMERS_H

#define TC0 TCCR0B
#define TC1 TCCR1B
#define TC2 TCCR2B
#define CLOCK_SELECT_NO_SOURCE      (0 << CS12) | (0 << CS11) | (0 << CS10)
#define CLOCK_SELECT_1_PRESCALER    (0 << CS12) | (0 << CS11) | (1 << CS10)
#define CLOCK_SELECT_8_PRESCALER    (0 << CS12) | (1 << CS11) | (0 << CS10)
#define CLOCK_SELECT_64_PRESCALER   (0 << CS12) | (1 << CS11) | (1 << CS10)
#define CLOCK_SELECT_256_PRESCALER  (1 << CS12) | (0 << CS11) | (0 << CS10)
#define CLOCK_SELECT_1024_PRESCALER (1 << CS12) | (0 << CS11) | (1 << CS10)

#define enableTimer(timer, clockSelect) timer = (timer & 0xF8) | clockSelect
#define disableTimer(timer) timer = (timer & 0xF8) | CLOCK_SELECT_NO_SOURCE

#endif