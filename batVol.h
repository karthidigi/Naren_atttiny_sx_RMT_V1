#include <avr/io.h>

uint16_t readVdd() {
  // Measure VDD by reading the internal 1.1 V bandgap against VDD as reference.
  // VDD (mV) = (1100 × 1023) / ADC_result
  ADC0.CTRLC = ADC_PRESC_DIV4_gc | ADC_REFSEL_VDDREF_gc;
  ADC0.MUXPOS = ADC_MUXPOS_INTREF_gc;
  VREF.CTRLA  = VREF_ADC0REFSEL_1V1_gc;
  ADC0.CTRLA  = ADC_ENABLE_bm;

  // Dummy conversion: bandgap needs ~100 µs to stabilise after ADC enable;
  // discard the first sample so subsequent reads are accurate.
  ADC0.COMMAND = ADC_STCONV_bm;
  while (!(ADC0.INTFLAGS & ADC_RESRDY_bm));
  ADC0.INTFLAGS = ADC_RESRDY_bm;

  // Average 4 samples to reduce ADC noise (~50 mV per single sample on battery).
  uint32_t sum = 0;
  for (uint8_t i = 0; i < 4; i++) {
    ADC0.COMMAND = ADC_STCONV_bm;
    while (!(ADC0.INTFLAGS & ADC_RESRDY_bm));
    sum += ADC0.RES;
    ADC0.INTFLAGS = ADC_RESRDY_bm;
  }

  ADC0.CTRLA = 0;  // disable ADC — saves ~130 µA on battery

  uint16_t result = (uint16_t)(sum >> 2);  // divide by 4
  if (result == 0) return 0;
  return (uint16_t)((1100UL * 1023UL) / result);
}

bool battCheck() {
  return readVdd() > BAT_VOL_MIN;
}