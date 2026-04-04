#include <avr/sleep.h>
#include <avr/interrupt.h>

static volatile bool lp_wakeup_flag = false;
static unsigned long lp_last_activity = 0;

// ISR – native PORT interrupt (ATtiny1606 PC2 = STA_BTN)
// PORTC_PORT_vect fires for any pin in PORT C.
ISR(PORTC_PORT_vect) {
  if (PORTC.INTFLAGS & PIN2_bm) {
    PORTC.INTFLAGS = PIN2_bm;   // clear flag (write 1 to clear)
    lp_wakeup_flag = true;
  }
}

static inline void lowPowerInit() {
  lp_last_activity = millis();
  // PC2 (STA_BTN) falling-edge interrupt — native PORT register
  // PULLUPEN preserved: hwPinInit() set INPUT_PULLUP; we keep it here explicitly.
  PORTC.DIRCLR   = PIN2_bm;                               // PC2 as input
  PORTC.PIN2CTRL = PORT_ISC_FALLING_gc | PORT_PULLUPEN_bm; // falling-edge + pull-up
}

static inline void lowPowerKick() {
  lp_last_activity = millis();
}

static inline void enterSleep() {
  // 1. Put SX1268 to sleep (warm start: retains calibration, faster wake)
  SX1268_setSleep(SX1268_SLEEP_WARM_START);

  // 2. Disable watchdog before sleep
  watchdogDisableFun();

  // 3. Enter deep sleep
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sei();            // enable interrupts before sleeping
  sleep_cpu();      // go to sleep — resumes at next line after ISR fires
  sleep_disable();

  // 4. Re-enable watchdog first so it covers the reinit below
  watchdogInit();

  // 5. Full radio reinit after deep sleep.
  // The SX1262 warm-start preserves registers, but the TCXO oscillator
  // requires re-calibration after extended power-down — especially when
  // ambient temperature has changed during sleep.  Without re-calibration
  // the RF carrier frequency drifts; at close range (strong RSSI) reception
  // still works, but at normal LoRa distance the frequency offset pushes the
  // signal off the receiver's passband and packets are lost.
  // A full sx1268Init() + switchToOperationalChannel() guarantees the radio
  // starts from a known, correctly calibrated state on the correct channel
  // with the correct sync word, SF12, and 22 dBm TX power.
  sx1268Init();
  switchToOperationalChannel();

  // 6. Re-enable ALL buttons so the user can send ON, OFF, or STA immediately
  // after waking without needing to press STA first to "unlock" the remote.
  buttonEn[0] = ENABLED;   // M1_ON
  buttonEn[1] = ENABLED;   // M1_OFF
  buttonEn[2] = ENABLED;   // STA

  // sx1268Init() already performed a hardware reset — no NSS toggle needed.
  lp_wakeup_flag = false;
  lp_last_activity = millis();
}

static inline void lowPowerPoll() {
  if ((millis() - lp_last_activity) >= LP_TIMEOUT_MS) {
    enterSleep();
  }
}
