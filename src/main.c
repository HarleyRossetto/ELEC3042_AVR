#include <avr/io.h>
#include <util/delay.h>

#define LED      PB5
#define LED_DDR  DDRB
#define LED_PORT PORTB

#define DELAYTIME 125

#define setBit(sfr, bit)     (_SFR_BYTE(sfr) |= (1 << bit))
#define clearBit(sfr, bit)   (_SFR_BYTE(sfr) &= ~(1 << bit))
#define toggleBit(sfr, bit)  (_SFR_BYTE(sfr) ^= (1 << bit))

int main(void) {
  DDRB |= (1 << PB2);
  PORTB |= (1 << PB2);

  while (1) {
      PORTB ^= (1 << PB2);
      _delay_ms(DELAYTIME);
  }

  return 0;                                      
}