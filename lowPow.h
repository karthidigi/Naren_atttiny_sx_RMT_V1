#include <avr/sleep.h>
#include <avr/interrupt.h>

static volatile bool lp_wakeup_flag = false;
static unsigned long lp_last_activity = 0;

// Set by enterSleep() after wake so hwbuttonFunc() fires [S?] on the first
// loop tick — even if the STA button is released before the scan runs.
// The STA falling edge that woke the MCU is consumed by the PORTC ISR before
// hwbuttonFunc() ever polls the pin; this flag carries the "STA caused the
// wake" information across the radio reinit delay.
bool lp_wkup_stbTx = false;

// ── STA_BTN wake ISR (native PORTC interrupt, PC2) ───────────────────────────
// Requires board attach=manual (CORE_ATTACH_NONE) so WInterrupts.c does not
// pre-define PORTC_PORT_vect.
ISR(PORTC_PORT_vect) {
  if (PORTC.INTFLAGS & PIN2_bm) {
    PORTC.INTFLAGS = PIN2_bm;   // write 1 to clear flag
    lp_wakeup_flag = true;
  }
}

static inline void lowPowerInit() {
  lp_last_activity = millis();
  // PC2 (STA_BTN) falling-edge interrupt — native PORT register
  // PULLUPEN preserved: hwPinInit() set INPUT_PULLUP; we keep it here explicitly.
  PORTC.DIRCLR   = PIN2_bm;
  PORTC.PIN2CTRL = PORT_ISC_FALLING_gc | PORT_PULLUPEN_bm;  // falling-edge + pull-up
}

static inline void lowPowerKick() {
  lp_last_activity = millis();
}

static inline void enterSleep() {
  // 0. Ensure all LEDs off before powering down.
  //    A cathode-sink LED left driven during sleep leaks ~2-5 mA continuously
  //    instead of the target ~3 µA, draining the battery during the sleep period.
  funcLedReset();

  // 1. Cancel any in-flight ACK wait — do NOT resume stale retries on wake.
  //    ackReception() checks msgTxd every 5 ms; leaving it 1 after sleep causes
  //    the device to fire retransmissions every 6 s as soon as it wakes.
  msgTxd        = 0;
  rxCrcError    = false;   // stale CRC flag would trigger a fast-retry (<500 ms) before ACK arrives
  lp_wkup_stbTx = false;  // clear any leftover flag before sleeping

  // 2. Put SX1268 to sleep (warm start: retains calibration, faster wake)
  SX1268_setSleep(SX1268_SLEEP_WARM_START);

  // 3. Disable watchdog before sleep — WDT cannot be serviced during PWR_DOWN
  watchdogDisableFun();

  // 4. Enter deep sleep (Power-Down: CPU + most clocks stopped; SRAM/registers retained)
  // PC2 (STA_BTN) falling-edge interrupt wakes the MCU — configured in lowPowerInit().
  // AVR guarantees the instruction after sei executes before any interrupt fires,
  // so sei + sleep_cpu is effectively atomic (no race on pending interrupt flag).
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sei();            // enable interrupts before sleeping
  sleep_cpu();      // go to sleep — resumes at next line after STA ISR fires
  sleep_disable();

  // 5. Re-enable watchdog; pet it immediately before long radio reinit.
  //    sx1268Init() runs TCXO setup + calibrate(0xFF) + image calibration —
  //    worst case ~500 ms+.  Without a WDT pet here the 4 s window can expire
  //    mid-init, causing a hard reset before the radio is ready.
  watchdogInit();
  watchdogReset();   // pet WDT before sx1268Init()

  // 6. Full radio reinit after deep sleep.
  //    The SX1262 warm-start preserves registers but the TCXO oscillator
  //    requires re-calibration after extended power-down — especially when
  //    ambient temperature has changed.  Without re-calibration the RF carrier
  //    drifts off the receiver's passband and packets are silently lost at
  //    normal range.  A full sx1268Init() + switchToOperationalChannel()
  //    guarantees a correctly calibrated radio on the right channel.
  sx1268Init();
  watchdogReset();   // pet WDT between init and channel switch
  switchToOperationalChannel();
  watchdogReset();   // pet WDT after all radio config

  // 7. Re-enable ALL buttons so the user can send ON, OFF, or STA immediately
  //    after waking without needing to press STA first to "unlock" the remote.
  buttonEn[0] = ENABLED;   // M1_ON
  buttonEn[1] = ENABLED;   // M1_OFF
  buttonEn[2] = ENABLED;   // STA

  // 8. Signal hwbuttonFunc() to fire [S?] on the very first loop tick.
  //    The STA falling edge that woke the MCU was consumed by the PORTC ISR —
  //    by the time hwbuttonFunc() runs (after the ~500 ms+ radio reinit above)
  //    the button is almost certainly released.  Without this flag the wake
  //    press is silently lost and the user must press STA a second time.
  lp_wakeup_flag = false;
  lp_wkup_stbTx  = true;
  lp_last_activity = millis();
}

static inline void lowPowerPoll() {
  if ((millis() - lp_last_activity) >= LP_TIMEOUT_MS) {
    enterSleep();
  }
}
