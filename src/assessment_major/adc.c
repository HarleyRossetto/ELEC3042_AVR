#include "adc.h"
#include "avr/interrupt.h"


Initialiser initialiseADC() {
    // ADC Multiplexer Selection Register
    ADMUX = ADC_VREF_AVCC | ADC_CH_0 | (1 << ADLAR);

    // ADC Control and Status Register A
    // Enabled ADC, Enable Interrupts, Enable Auto Trigger, 128 prescaler
    ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADATE) | (ADC_PRESCALER_128);

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

uint8_t adc_value = 0;

// ADC - Conversion Complete
ISR(ADC_vect) { adc_value = ADCH; }