#define NUM_BUTTONS 3
static const uint8_t buttonPins[NUM_BUTTONS] = { M1_ON_BTN, M1_OFF_BTN, STA_BTN };
static uint8_t buttonStates[NUM_BUTTONS]     = { 1, 1, 1 };
static uint8_t lastButtonStates[NUM_BUTTONS] = { 1, 1, 1 };
static unsigned long lastDebounceTimes[NUM_BUTTONS] = { 0, 0, 0 };
static const unsigned long debounceDelay = 50UL;
unsigned long ackTimerMillis = 0;
uint8_t ackFailAtmp = 0;
// (lastTxCmdCode moved to states.h — remProto.h needs it for the OFF-pre-empt guard)

// Forward declaration — enterRemNodePairMode() is defined in pairRemoteNode.h
// which is included after button.h in the .ino
void enterRemNodePairMode();

// ── STA-as-modifier (light-switch chord) state ───────────────────────────────
// STA (index 2) is a modifier: while it is held, M1_ON/M1_OFF become the light
// commands REM_CMD_RELAY_ON/OFF (and the motor command is suppressed). A lone
// STA tap still sends STATUS (deferred to release). This is the safety guard so
// a motor STOP only ever fires when STA is NOT held, and a light action only
// ever fires when STA IS held — the two can never cross.
static bool staActive   = false;  // STA currently held (debounced)
static bool staConsumed = false;  // a chord fired during this STA hold

// Centralised command TX: send, arm the retry timer. ON(0)/STA(2) are disabled
// for the ACK window (redundant with msgTxd but kept for the wake/chord gates);
// OFF(1) is deliberately LEFT ENABLED so it can pre-empt the pending command —
// once paired, the stop key must always work. Calling this while msgTxd is
// already set simply REPLACES the pending command (fresh retry state).
static inline void remTxButtonCmd(uint8_t cmd) {
  buttonEn[0] = DISABLED;
  buttonEn[2] = DISABLED;
  lastTxCmdCode  = cmd;
  remSendCmdNew(cmd);
  msgTxd         = 1;
  ackFailAtmp    = 0;
  ackTimerMillis = millis();
}

static inline void hwbuttonFunc() {
  // ── STA-wake TX ──────────────────────────────────────────────────────────────
  // The STA falling edge that woke the MCU from deep sleep is consumed by the
  // PORTC ISR before hwbuttonFunc() ever polls the pin. lp_wkup_stbTx carries the
  // "STA caused the wake" info so the STATUS TX is never lost. (A STA+OFF chord
  // from deep sleep sends STATUS on wake; re-press while awake for light control.)
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
    staActive   = false;   // wake consumed STA; no chord in progress
    staConsumed = false;
    remTxButtonCmd(REM_CMD_STATUS);
    return;
  }
  lp_wkup_stbTx = false;

  // ── Debounce all three buttons first; record stable edges ────────────────────
  // Debouncing every button before acting means a STA+OFF press resolves STA's
  // held-state in the same pass, so the OFF edge can never be misread as a
  // standalone motor-off when STA is genuinely held.
  uint8_t edgeLow[NUM_BUTTONS]  = { 0, 0, 0 };
  uint8_t edgeHigh[NUM_BUTTONS] = { 0, 0, 0 };
  for (uint8_t i = 0; i < NUM_BUTTONS; ++i) {
    uint8_t reading = digitalRead(buttonPins[i]);
    if (reading != lastButtonStates[i]) lastDebounceTimes[i] = millis();
    if ((millis() - lastDebounceTimes[i]) > debounceDelay) {
      if (reading != buttonStates[i]) {
        buttonStates[i] = reading;
        if (reading == LOW) edgeLow[i] = 1; else edgeHigh[i] = 1;
      }
    }
    lastButtonStates[i] = reading;
  }

  // ── STA modifier tracking (index 2) ──────────────────────────────────────────
  if (edgeLow[2] && buttonEn[2] == ENABLED) {
    staActive   = true;
    staConsumed = false;
    lowPowerKick();
  }
  if (edgeHigh[2]) {
    // STA released: if it was a lone tap (no chord consumed it), send STATUS now.
    if (staActive && !staConsumed && buttonEn[2] == ENABLED && !msgTxd) {
      watchdogReset();
      lowPowerKick();
      funcStaLBlue(); buzBeep(50); funcLedReset();
      remTxButtonCmd(REM_CMD_STATUS);
    }
    staActive   = false;
    staConsumed = false;
  }

  // ── Motor / light buttons (indices 0 = ON, 1 = OFF) ──────────────────────────
  // Always-enabled motor buttons (no starter-status gating):
  //   • OFF — enabled whenever the remote is PAIRED (buttonEn[1] set at boot/
  //     pairing, never cleared by motor status) and PRE-EMPTS an in-flight
  //     command: pressing OFF while an ON/STATUS awaits its ACK abandons that
  //     wait and TXes M1_OFF immediately — stop always wins.
  //   • ON  — also independent of last-known motor status, but it must NOT
  //     pre-empt: while any command is in flight (msgTxd) ON is ignored, so a
  //     pending OFF can never be overridden by a start.
  //   • Light chord (STA held) — unchanged: needs an idle link (!msgTxd).
  // The starter is idempotent + time-window-deduped on both commands, so a
  // redundant ON/OFF is answered from cache without re-actuating the relay.
  for (uint8_t i = 0; i < 2; ++i) {
    if (!edgeLow[i]) continue;

    if (staActive) {
      // STA held → LIGHT command (chord). NOT gated by the motor buttonEn[] so
      // the light can be toggled regardless of motor on/off state. The motor
      // command for this key press is suppressed.
      if (msgTxd) continue;
      watchdogReset();
      lowPowerKick();
      funcStaLBlue(); buzBeep(50); funcLedReset();
      while (digitalRead(buttonPins[i]) == LOW) { watchdogReset(); delay(1); }
      staConsumed = true;
      remTxButtonCmd(i == 0 ? REM_CMD_RELAY_ON : REM_CMD_RELAY_OFF);
      break;
    } else if (i == 1 && buttonEn[1] == ENABLED
               && (!msgTxd
                   || lastTxCmdCode == REM_CMD_M1_ON
                   || lastTxCmdCode == REM_CMD_M1_OFF
                   || lastTxCmdCode == REM_CMD_STATUS)) {
      // OFF — allowed even mid-ACK-wait (pre-empt): remTxButtonCmd replaces the
      // pending command and restarts the retry state for M1_OFF.
      // Pre-empt is limited to MOTOR/STATUS in-flight commands: a RELAY light
      // chord must settle first, otherwise its crossed RSP (status=OFF) would be
      // mis-read as the OFF's ACK and cancel the OFF retry (review finding 7).
      // The RELAY ack window is short (~2×ToA+0.9 s), so the wait is brief.
      watchdogReset();
      lowPowerKick();
      funcStaLBlue(); buzBeep(50); funcLedReset();
      while (digitalRead(buttonPins[i]) == LOW) { watchdogReset(); delay(1); }
      remTxButtonCmd(REM_CMD_M1_OFF);
      break;
    } else if (i == 0 && !msgTxd && buttonEn[0] == ENABLED) {
      // ON — no pre-empt; waits for an idle link.
      watchdogReset();
      lowPowerKick();
      funcStaLBlue(); buzBeep(50); funcLedReset();
      while (digitalRead(buttonPins[i]) == LOW) { watchdogReset(); delay(1); }
      remTxButtonCmd(REM_CMD_M1_ON);
      break;
    }
  }
}

// Per-command ack-wait window. Motor ON/OFF wait for the starter's deferred CT-current
// confirmation (long); STATUS/relay use the short normal round-trip so a dropped packet
// recovers fast. See ACK_WAIT_*_MS in zSettings.h for the derivation.
static inline unsigned long ackWaitFor(uint8_t cmd) {
  if (cmd == REM_CMD_M1_ON)  return ACK_WAIT_ON_MS;
  if (cmd == REM_CMD_M1_OFF) return ACK_WAIT_OFF_MS;
  return ACK_WAIT_STATUS_MS;   // STATUS, RELAY_ON/OFF, and default
}

////////////////////////////////////////
void ackReception() {
  if (msgTxd) {
    lowPowerKick();
    unsigned long ackWait = ackWaitFor(lastTxCmdCode);

    if (rxCrcError || rxHdrError) {
      // A reply reached us but failed CRC or header decode — don't wait the full window;
      // retry ~500 ms from now. Offset scales with the per-command ack-wait.
      rxCrcError = false;
      rxHdrError = false;
      ackTimerMillis = millis() - (ackWait - 500UL);
    }

    if (millis() - ackTimerMillis > ackWait) {
      if (ackFailAtmp >= 3) {
        watchdogReset();   // pet WDT before tone — tone timing is ~0.5 s
        noNetworkTone();
        watchdogReset();   // pet again: noTone() restores TCA0 but millis() may lag
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
