#include "adc.h"
#include "avr/interrupt.h"

Initialiser initialiseADC() {
    // ADC Multiplexer Selection Register
    ADMUX = ADC_VREF_AVCC | ADC_CH_0 | (1 << ADLAR);

    // ADC Control and Status Register A
    // Enabled ADC, Enable Interrupts, Enable Auto Trigger, 128 prescaler
    ADCSRA = (1 << ADEN) | (1 << ADIE);

    // ADC Control and Status Register B
    ADCSRB = 0x00;

    // ADC Data Register
    ADC = 0;

    // Digital Input Disable Register
    DIDR0 = (1 << ADC0D); // Disable digital input on AD0
}


inline void ADC_StartConversion() {
     ADCSRA |= (1 << ADSC);
}

volatile uint16_t runningAverage;
volatile uint16_t adcAverageValue = 0;
uint16_t ADC_GetSampleAverage() { return adcAverageValue; }

#define ADC_AVERAGE_COUNT 5
static uint8_t averagesTaken = 0;
// ADC - Conversion Complete
ISR(ADC_vect) { 
    // Accumulate value
    runningAverage += ADCH;
    averagesTaken++;

    if (averagesTaken == ADC_AVERAGE_COUNT) {
        adcAverageValue = runningAverage / ADC_AVERAGE_COUNT;
        runningAverage  = 0;
        averagesTaken = 0;
    }
}