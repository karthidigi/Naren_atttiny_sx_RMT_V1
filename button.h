#define AES_KEY_LEN 16

#define NUM_BUTTONS 3
static const uint8_t buttonPins[NUM_BUTTONS] = { M1_ON_BTN, M1_OFF_BTN, STA_BTN };
static uint8_t buttonStates[NUM_BUTTONS]     = { 1, 1, 1 };
static uint8_t lastButtonStates[NUM_BUTTONS] = { 1, 1, 1 };
static unsigned long lastDebounceTimes[NUM_BUTTONS] = { 0, 0, 0 };
static const unsigned long debounceDelay = 50UL;
unsigned long ackTimerMillis = 0;
uint8_t ackFailAtmp = 0;
static char lastTxCmd[8] = {0};  // original command to retry on ACK failure

void encryptNTx(const char *msg) {
  encryptData(msg);
  send_lora_data((uint8_t *)txBuffer, 32);
}

// Forward declaration — enterRemNodePairMode() is defined in pairRemoteNode.h
// which is included after button.h in the .ino
void enterRemNodePairMode();

static inline void hwbuttonFunc() {
  // Runtime re-pair combo REMOVED for v2-2 board.
  // Re-pair is triggered by boot combo only (M1_OFF + STA >= 5s in setup()).

  // ── STA-wake TX ──────────────────────────────────────────────────────────────
  // The STA falling edge that woke the MCU from deep sleep is consumed by the
  // PORTC ISR before hwbuttonFunc() ever polls the pin — by the time we get
  // here (after sx1268Init() + switchToOperationalChannel() which take ~500 ms+)
  // the button is almost certainly already released.  lp_wkup_stbTx carries the
  // "STA caused the wake" information so the TX is never silently lost.
  if (lp_wkup_stbTx && !msgTxd && buttonEn[2] == ENABLED) {
    lp_wkup_stbTx = false;
    lowPowerKick();
    funcStaLBlue(); buzBeep(50); funcLedReset();
    // STA button may still be physically held here.  Wait for release then reset
    // all button debounce state so the regular scan below does NOT detect a
    // second falling edge and fire a duplicate TX.  A duplicate TX overrides
    // STATE_TX_WAIT mid-transmit and corrupts the radio.
    while (digitalRead(buttonPins[2]) == LOW) {
      watchdogReset();
      delay(1);
    }
    delay(debounceDelay);   // 50 ms RC-settle after release
    for (uint8_t j = 0; j < NUM_BUTTONS; j++) {
      buttonStates[j]      = 1;   // HIGH = released
      lastButtonStates[j]  = 1;
      lastDebounceTimes[j] = millis();
    }
    strncpy(lastTxCmd, "[S?]", sizeof(lastTxCmd));
    encryptNTx("[S?]");
    msgTxd         = 1;
    ackFailAtmp    = 0;
    ackTimerMillis = millis();
    return;   // skip regular button scan this tick
  }
  lp_wkup_stbTx = false;   // clear if buttons not yet enabled (e.g. still pairing)

  for (uint8_t i = 0; i < NUM_BUTTONS; ++i) {
    uint8_t reading = digitalRead(buttonPins[i]);
    if (reading != lastButtonStates[i]) lastDebounceTimes[i] = millis();
    if ((millis() - lastDebounceTimes[i]) > debounceDelay) {
      if (reading != buttonStates[i]) {
        buttonStates[i] = reading;

        if ((buttonStates[i] == LOW) && (buttonEn[i] == ENABLED)) {
          watchdogReset();
          lowPowerKick();
          // unsigned long pressStart = millis();  // reserved for long-press detection (future use)
          funcStaLBlue();       // blue LED ON
          buzBeep(50);          // buzzer + LED feedback — 50 ms
          funcLedReset();       // blue LED OFF (buzzer already off after buzBeep)
          while (digitalRead(buttonPins[i]) == LOW) {
            delay(1);  // small delay to avoid CPU hogging
          }

          // Disable all buttons during TX
          for (uint8_t j = 0; j < NUM_BUTTONS; j++) buttonEn[j] = DISABLED;

          delay(100);

          switch (i) {
            case 0:
              strncpy(lastTxCmd, "[1N]", sizeof(lastTxCmd));
              encryptNTx("[1N]");
              break;
            case 1:
              strncpy(lastTxCmd, "[1F]", sizeof(lastTxCmd));
              encryptNTx("[1F]");
              break;
            case 2: {
              // Long-press light-switch toggle (≥300 ms → L1/L0) is reserved for
              // future use — commented out until the feature is fully implemented.
              // unsigned long heldMs = millis() - pressStart;
              // if (heldMs >= 300UL) {
              //   if (liRelRemState) {
              //     strncpy(lastTxCmd, "[L0]", sizeof(lastTxCmd));
              //     encryptNTx("[L0]");
              //   } else {
              //     strncpy(lastTxCmd, "[L1]", sizeof(lastTxCmd));
              //     encryptNTx("[L1]");
              //   }
              // } else {

              // Short press — status query (always, while light-switch is disabled)
              strncpy(lastTxCmd, "[S?]", sizeof(lastTxCmd));
              encryptNTx("[S?]");

              // } // end long-press else
              break;
            }
          }

          msgTxd = 1;
          ackFailAtmp = 0;
          ackTimerMillis = millis();
        }
      }
    }

    lastButtonStates[i] = reading;
  }
}

////////////////////////////////////////
void ackReception() {
  if (msgTxd) {
    lowPowerKick();   // prevent LP sleep while waiting for ACK reply

    // Fast retry on CRC error: packet arrived but payload was corrupted.
    // Reset the ACK timer so the retry fires in ~500 ms instead of 6 s.
    // Unsigned subtraction wraps correctly: millis() - ackTimerMillis will
    // equal 5500 immediately after this assignment, reaching >6000 after ~500 ms.
    if (rxCrcError) {
      rxCrcError = false;
      ackTimerMillis = millis() - 5500UL;
    }

    // SF11 full ACK cycle: [S?] ToA ~1.1s + starter delay 1.5s + ACK ToA ~1.1s = ~3.7s total.
    // 6000ms gives 2.3s margin — enough for the full ACK to arrive before declaring a retry.
    if (millis() - ackTimerMillis > 6000) {
      if (ackFailAtmp >= 3) {
        noNetworkTone();
        funcStaLPink();    // no-network indicator: pink (red + blue)
        delay(300);
        funcLedReset();
        delay(300);
        funcStaLPink();
        delay(300);
        funcLedReset();
        // Radio re-init: after repeated TX failures the radio may be stuck in a
        // bad state (TX latch, BUSY stuck, IRQ flags not cleared).
        // Full reinit + return to operational channel ensures clean RX on next press.
        sx1268Init();
        switchToOperationalChannel();
        // Re-enable ALL buttons — not just STA.
        // Previous behaviour (STA only) confused users: after returning to range
        // ON/OFF presses did nothing because those buttons were still locked out.
        buttonEn[0] = ENABLED;   // M1_ON
        buttonEn[1] = ENABLED;   // M1_OFF
        buttonEn[2] = ENABLED;   // STA
        msgTxd = 0;
        // Reset the low-power sleep timer.  The retry loop takes ~32 s, which
        // is just over LP_TIMEOUT_MS, so without this the device enters sleep
        // on the very next lowPowerPoll() call before the user can press anything.
        lowPowerKick();
      } else {
        funcStaLBlue();
        delay(300);
        funcLedReset();
        // Retry with the original command — not always [S?].
        // If [1N] or [1F] was lost in transit, the retry must repeat
        // the command so the motor actually gets commanded on success.
        encryptNTx(lastTxCmd[0] ? lastTxCmd : "[S?]");
        ackFailAtmp++;
      }
      ackTimerMillis = millis();
    }
  }
}
