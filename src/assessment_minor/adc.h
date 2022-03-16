#ifndef ADC_H
#define ADC_H

#define ADC_VREF_OFF (0 << REFS1) | (0 << REFS0)
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

#endif //ADC_H