// led.h — v2-2 board LED driver
// Topology: cathode-sink — all LEDs are driven as cathode sinks.
// Drive pin OUTPUT LOW = LED on, DIRCLR (INPUT high-Z) = LED off.
// PB3 = blue cathode, PC1 = red cathode, PB4 = green cathode.

void funcLedReset() {
  PORTC.DIRCLR = PIN1_bm;             // red (PC1) → high-Z = off
  PORTB.DIRCLR = PIN3_bm | PIN4_bm;  // blue (PB3) + green (PB4) → high-Z = off
}

void funcStaLBlue() {
  funcLedReset();
  PORTB.OUTCLR = PIN3_bm;   // PB3 LOW → blue LED on
  PORTB.DIRSET = PIN3_bm;
}

void funcStaLRed() {
  funcLedReset();
  PORTC.OUTCLR = PIN1_bm;   // PC1 LOW → red LED on
  PORTC.DIRSET = PIN1_bm;
}

// Orange (red + green): fault / alarm — F1, D1, A0
void funcStaLOrange() {
  funcLedReset();
  PORTC.OUTCLR = PIN1_bm;   // red on (PC1)
  PORTB.OUTCLR = PIN4_bm;   // green on (PB4) → red + green = orange/amber
  PORTC.DIRSET = PIN1_bm;
  PORTB.DIRSET = PIN4_bm;
}

// Cyan (blue + green): bypass / override — B1, D2
void funcStaLCyan() {
  funcLedReset();
  PORTB.OUTCLR = PIN3_bm | PIN4_bm;  // blue (PB3) + green (PB4) = cyan
  PORTB.DIRSET = PIN3_bm | PIN4_bm;
}

// Pink (red + blue): no network — ackFailAtmp >= 3
void funcStaLPink() {
  funcLedReset();
  PORTC.OUTCLR = PIN1_bm;   // red on (PC1)
  PORTB.OUTCLR = PIN3_bm;   // blue on (PB3) → red + blue = pink
  PORTC.DIRSET = PIN1_bm;
  PORTB.DIRSET = PIN3_bm;
}

void funcStaLWhite() {
  funcLedReset();
  PORTC.OUTCLR = PIN1_bm;             // red on (PC1)
  PORTB.OUTCLR = PIN3_bm | PIN4_bm;  // blue (PB3) + green (PB4) on → all = white
  PORTC.DIRSET = PIN1_bm;
  PORTB.DIRSET = PIN3_bm | PIN4_bm;
}

void funcM1LRed() {
  funcStaLRed();             // same cathode as status red
}

void funcM1Yellow() {
  funcLedReset();
  PORTC.OUTCLR = PIN1_bm;   // red cathode on
  PORTB.OUTCLR = PIN4_bm;   // green cathode on → together = yellow
  PORTC.DIRSET = PIN1_bm;
  PORTB.DIRSET = PIN4_bm;
}

void lowBattAlert() {
  // 5 × red blink — called before buzzer beeps which are added at the call site
  // (buz.h is included after led.h so buzBeep is not visible here).
  for (uint8_t i = 0; i < 5; i++) {
    funcStaLRed(); delay(200); funcLedReset(); delay(150);
  }
}
