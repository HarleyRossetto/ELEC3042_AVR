/**
 * @file adc.h
 * @author Harley Rossetto (44618883@students.mq.edu.au)
 * @brief Macros to help configure the ATmega328p's ADC module.
 * @version 0.1
 * @date 2022-04-06
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef ADC_H
#define ADC_H

#define ADC_VREF_OFF  (0 << REFS1) | (0 << REFS0)
#define ADC_VREF_AVCC (0 << REFS1) | (1 << REFS0)
#define ADC_VREF_AREF (1 << REFS1) | (1 << REFS0)

#define ADC_CH_0 0b0000
#define ADC_CH_1 0b0001
#define ADC_CH_2 0b0010
#define ADC_CH_3 0b0011
#define ADC_CH_4 0b0100
#define ADC_CH_5 0b0101
#define ADC_CH_6 0b0110
#define ADC_CH_7 0b0111
#define ADC_CH_8 0b1000

#define ADC_PRESCALER_32 ((1 << ADPS2) | (0 << ADPS1) | (1 << ADPS0))
#define ADC_PRESCALER_64 ((1 << ADPS2) | (1 << ADPS1) | (0 << ADPS0))
#define ADC_PRESCALER_128 ((1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0))

#define ADC_AUTO_TRIGGER_SOURCE_TCC0_COMPARE_MATCH_A ((0 << ADTS2) | (1 << ADTS1) | (1 << ADTS0))
#define ADC_AUTO_TRIGGER_FREE_RUNNING ((0 << ADTS2) | (0 << ADTS1) | (0 << ADTS0))

// Aalias OCR0A (where ADCH is placed upon comparison)
#define ADC_VALUE OCR0A 

#endif // ADC_H