
#include <util/delay.h>   // _delay_ms() — F_CPU-based NOP loop, no timer dependency

// ---------------- NOTE DEFINITIONS ----------------

#define NOTE_A4  440
#define NOTE_F4  349
#define NOTE_C5  523
#define NOTE_E5  659
#define NOTE_G5  784
#define NOTE_C6 1047
#define NOTE_E6 1319

// Motor ON confirmed — "Victory Fanfare" ~2 s.
// Hardware tone() drives the buzzer; no software PWM while-loop.
// Green LED blinks every 150 ms throughout the tone duration.
// Call watchdogReset() before invoking (WDT = 4 s, tone ≈ 2 s).
void motorOnTone() {
  const int     mel[] = { NOTE_C5, NOTE_E5, NOTE_G5, NOTE_C6,
                           NOTE_E6, NOTE_C6, NOTE_G5, NOTE_C6 };
  const uint8_t dur[] = { 10, 10, 10, 10, 7, 10, 10, 3 };
  // note ms (1000/div): 100 100 100 100 142 100 100 333 = 1075 ms
  // pauses  (30% each):  30  30  30  30  42  30  30  99 =  321 ms
  // total ≈ 1396 ms ≈ 1.4 s

  // ── Why _delay_ms(1) instead of millis() or delay() ───────────────────────
  // tone() on PIN_PB0 uses TCA0 lower half (WO0). It reprograms LPER to the
  // audio frequency so the TCA0 lower-half underflow ISR (LUNF) — which drives
  // millis() — fires several times faster than real time.  A 1047 Hz note makes
  // millis() run ~4× fast; the 150 ms LED blink becomes ~35 ms real time
  // (~14 Hz flicker) which looks like a faded/dim LED.
  // delay() internally calls millis(), so it is equally broken during tone().
  // _delay_ms(1) is a pure F_CPU-based NOP loop with zero timer dependency —
  // it is always accurate regardless of TCA0 state.
  //
  // Also clear TCA0 WO4 override on PB4 (green LED) so no stale analogWrite()
  // can drive PB4 as PWM while we use it as a direct PORT output.
  TCA0.SPLIT.CTRLB &= ~TCA_SPLIT_HCMP1EN_bm;

  // Direct PORT writes — avoids funcM1LGreen() calling funcLedReset() which
  // briefly floats PB4 as high-Z on every ON half of the blink.
  bool ledState = true;
  PORTB.OUTCLR = PIN4_bm;   // output register LOW (cathode sink = LED on)
  PORTB.DIRSET = PIN4_bm;   // drive → LED ON

  // ledTick counts ms across ALL notes — must live outside the note loop so
  // it is NOT reset at the start of each note.  When it was inside the loop,
  // every 100 ms note reset it to 0 before it ever reached 150, so the LED
  // never toggled.
  uint16_t ledTick = 0;

  size_t length = sizeof(mel) / sizeof(mel[0]);
  for (size_t i = 0; i < length; i++) {
    uint16_t noteDuration  = 1000U / dur[i];           // ms
    uint16_t pauseDuration = (noteDuration * 3U) / 10U; // ms

    // No duration argument — tone() with duration uses millis() internally to
    // stop the note; corrupted millis() cuts every note short and changes the
    // melody.  Run indefinitely and stop it ourselves via noTone() below.
    tone(BUZ, (unsigned int)mel[i]);

    // Count 1 ms ticks with _delay_ms(1) for both note and pause timing.
    // LED toggles every 150 ticks (150 ms real time).
    uint16_t totalMs = noteDuration + pauseDuration;

    for (uint16_t t = 0; t < totalMs; t++) {
      _delay_ms(1);

      if (t == noteDuration) {
        noTone(BUZ);
        digitalWrite(BUZ, HIGH);   // silence active-low buzzer during pause
      }

      if (++ledTick >= 150) {
        ledTick = 0;
        ledState = !ledState;
        if (ledState) {
          PORTB.OUTCLR = PIN4_bm;
          PORTB.DIRSET = PIN4_bm;   // LED ON
        } else {
          PORTB.DIRCLR = PIN4_bm;   // high-Z = LED OFF
        }
      }
    }
  }

  noTone(BUZ);
  digitalWrite(BUZ, HIGH);
  PORTB.DIRCLR = PIN4_bm;   // LED off
}

void noNetworkTone() {
  const int     mel[] = { NOTE_C5, NOTE_A4, NOTE_F4 };
  const uint8_t dur[] = { 16, 16, 8 };

  size_t length = sizeof(mel) / sizeof(mel[0]);
  for (size_t i = 0; i < length; i++) {
    unsigned long noteDuration = 1000UL / dur[i];
    tone(BUZ, (unsigned int)mel[i], noteDuration);  // hardware tone — no while-loop
    delay(noteDuration + noteDuration / 5);          // note duration + 20% pause
  }
  noTone(BUZ);
  digitalWrite(BUZ, HIGH);  // Ensure idle
}

// Buzzer: short blocking beep only for small durations
static inline void buzBeep(uint16_t ms) {
  digitalWrite(BUZ, LOW);
  delay(ms);
  digitalWrite(BUZ, HIGH);
}
