#define NUM_BUTTONS 3
static const uint8_t buttonPins[NUM_BUTTONS] = { M1_ON_BTN, M1_OFF_BTN, STA_BTN };
static uint8_t buttonStates[NUM_BUTTONS]     = { 1, 1, 1 };
static uint8_t lastButtonStates[NUM_BUTTONS] = { 1, 1, 1 };
static unsigned long lastDebounceTimes[NUM_BUTTONS] = { 0, 0, 0 };
static const unsigned long debounceDelay = 50UL;
unsigned long ackTimerMillis = 0;
uint8_t ackFailAtmp = 0;
static uint8_t lastTxCmdCode = 0;  // REM_CMD_* of the last new command; 0 = none

// Forward declaration — enterRemNodePairMode() is defined in pairRemoteNode.h
// which is included after button.h in the .ino
void enterRemNodePairMode();

static inline void hwbuttonFunc() {
  // ── STA-wake TX ──────────────────────────────────────────────────────────────
  // The STA falling edge that woke the MCU from deep sleep is consumed by the
  // PORTC ISR before hwbuttonFunc() ever polls the pin — by the time we get here
  // (after sx1268Init() + switchToOperationalChannel() which take ~500 ms+) the
  // button is almost certainly already released.  lp_wkup_stbTx carries the
  // "STA caused the wake" information so the TX is never silently lost.
  if (lp_wkup_stbTx && !msgTxd && buttonEn[2] == ENABLED) {
    lp_wkup_stbTx = false;
    lowPowerKick();
    funcStaLBlue(); buzBeep(50); funcLedReset();
    while (digitalRead(buttonPins[2]) == LOW) {
      watchdogReset();
      delay(1);
    }
    delay(debounceDelay);
    for (uint8_t j = 0; j < NUM_BUTTONS; j++) {
      buttonStates[j]      = 1;
      lastButtonStates[j]  = 1;
      lastDebounceTimes[j] = millis();
    }
    lastTxCmdCode = REM_CMD_STATUS;
    remSendCmdNew(REM_CMD_STATUS);
    msgTxd         = 1;
    ackFailAtmp    = 0;
    ackTimerMillis = millis();
    return;
  }
  lp_wkup_stbTx = false;

  for (uint8_t i = 0; i < NUM_BUTTONS; ++i) {
    uint8_t reading = digitalRead(buttonPins[i]);
    if (reading != lastButtonStates[i]) lastDebounceTimes[i] = millis();
    if ((millis() - lastDebounceTimes[i]) > debounceDelay) {
      if (reading != buttonStates[i]) {
        buttonStates[i] = reading;

        if ((buttonStates[i] == LOW) && (buttonEn[i] == ENABLED)) {
          watchdogReset();
          lowPowerKick();
          funcStaLBlue(); buzBeep(50); funcLedReset();
          while (digitalRead(buttonPins[i]) == LOW) {
            delay(1);
          }

          for (uint8_t j = 0; j < NUM_BUTTONS; j++) buttonEn[j] = DISABLED;
          delay(100);

          switch (i) {
            case 0:
              lastTxCmdCode = REM_CMD_M1_ON;
              remSendCmdNew(REM_CMD_M1_ON);
              break;
            case 1:
              lastTxCmdCode = REM_CMD_M1_OFF;
              remSendCmdNew(REM_CMD_M1_OFF);
              break;
            case 2:
              lastTxCmdCode = REM_CMD_STATUS;
              remSendCmdNew(REM_CMD_STATUS);
              break;
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
    lowPowerKick();

    if (rxCrcError) {
      rxCrcError = false;
      ackTimerMillis = millis() - 5500UL;
    }

    if (millis() - ackTimerMillis > 6000) {
      if (ackFailAtmp >= 3) {
        watchdogReset();   // ~3.4 s of blocking buzzer + LED ahead — pet WDT first
        noNetworkTone();
        funcStaLPink();
        delay(300);
        funcLedReset();
        delay(300);
        funcStaLPink();
        delay(300);
        funcLedReset();
        sx1268Init();
        switchToOperationalChannel();
        buttonEn[0] = ENABLED;
        buttonEn[1] = ENABLED;
        buttonEn[2] = ENABLED;
        msgTxd = 0;
        lowPowerKick();
      } else {
        funcStaLBlue();
        delay(300);
        funcLedReset();
        remSendCmd(lastTxCmdCode ? lastTxCmdCode : REM_CMD_STATUS);
        ackFailAtmp++;
      }
      ackTimerMillis = millis();
    }
  }
}
