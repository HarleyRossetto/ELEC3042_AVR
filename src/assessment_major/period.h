#if !defined(PERIOD_H)
#define PERIOD_H

#include "adc.h"

// ADC value to Period Time MS Lookup table.
extern const uint16_t ADC_TO_PERIOD_TIME_MS_LUT[256];

#define PERIOD_MS ADC_TO_PERIOD_TIME_MS_LUT[adc_value]

#endif // PERIOD_H
