#include "adc_freerunner.h"

#include <avr/interrupt.h>

uint16_t volatile adc_values[ADC_CHANNELS];

void setup_adc() {
  ADMUX |= (1 << REFS0); // Set ADC reference to AVCC
  ADCSRA |= (1 << ADPS2) | (0 << ADPS1) | (1 << ADPS0);
  ADCSRA |= (1 << ADATE);
  ADCSRA |= (1 << ADIE);  // enable ADC interrupt
  ADCSRA |= (1 << ADEN);  // Enable ADC
  ADCSRA |= (1 << ADSC);  // Start A2D Conversions

  //    sei();
}

//ISR(ADC_vect) {
//  static uint8_t oldchan;
//  static uint8_t counter;
//  static uint16_t adc_buffer[ADC_CHANNELS];
//  uint8_t curchan = ADMUX & 7;
//  adc_buffer[oldchan] += ADCL | (ADCH << 8);
//  oldchan = curchan;
//  if (++curchan == ADC_CHANNELS) {
//    curchan = 0;
//    if (++counter == ADC_OVERSAMPLING) {
//      counter = 0;
//      for (uint8_t i = 0; i < ADC_CHANNELS; ++i) {
//        adc_values[i] = adc_buffer[i];
//        adc_buffer[i] = 0;
//      }
//    }
//  }
//  ADMUX = (ADMUX & ~7) | curchan;
//}
