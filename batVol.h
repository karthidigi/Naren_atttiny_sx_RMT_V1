#include <avr/io.h>

uint16_t readVdd() {
  // Measure VDD by reading the internal 1.1 V bandgap against VDD as reference.
  //   VDD (mV) = (1100 × 1023) / ADC_result
  //
  // Timing / accuracy notes (ATtiny1606) — getting these wrong makes a healthy
  // battery read as critically low and falsely fires lowBattAlert() at boot:
  //   • CLK_ADC MUST be ≤ 1.5 MHz for a valid conversion.  With megaTinyCore the
  //     core clock is often 20 MHz, so the old DIV4 prescaler gave 5 MHz —
  //     out of spec → garbage/unstable results.  DIV64 → ≤ 312 kHz @ 20 MHz.
  //   • The 1.1 V bandgap needs ~25–100 µs to start up after the ADC/reference is
  //     enabled.  A single ~15-clock dummy conversion is far too short; if the
  //     reference is still near 0 V the result collapses and computed VDD reads
  //     near 0 → false low battery.  Wait explicitly with delayMicroseconds().
  //   • SAMPCAP must be 1 for any reference ≥ 1 V (VDD here) — wrong setting adds
  //     gain error.
  //   • The bandgap is a high-impedance source: extend SAMPCTRL so the S/H cap
  //     fully charges.
  ADC0.CTRLC    = ADC_PRESC_DIV64_gc | ADC_REFSEL_VDDREF_gc | ADC_SAMPCAP_bm;
  ADC0.MUXPOS   = ADC_MUXPOS_INTREF_gc;
  VREF.CTRLA    = VREF_ADC0REFSEL_1V1_gc;
  ADC0.SAMPCTRL = 31;                 // max extra sample length for high-Z bandgap
  ADC0.CTRLA    = ADC_ENABLE_bm;      // 10-bit, single-ended

  // Reference + bandgap startup: explicit settle (the old 1-shot dummy was ~µs,
  // not the ~100 µs the bandgap actually needs).
  delayMicroseconds(200);

  // Dummy conversion: discard the first sample after settling.
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
  // Debounced: a single transient sag (radio TX inrush) or one anomalous ADC
  // reading must NEVER declare a low battery.  Return healthy as soon as any of
  // a few reads is above the threshold; only call it low when every read is low.
  for (uint8_t i = 0; i < 3; i++) {
    if (readVdd() > BAT_VOL_MIN) return true;   // confirmed healthy — stop early
  }
  return false;                                 // all reads low → genuinely low
}
