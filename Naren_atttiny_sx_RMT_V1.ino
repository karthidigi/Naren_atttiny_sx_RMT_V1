#include "zSettings.h"
#include "hwPins.h"
#include "led.h"
#include "batVol.h"
#include "buz.h"
#include "debug.h"
#include "states.h"
#include "watDog.h"
#include "eeprom.h"
#include "aesMain.h"
#include "devId.h"
#include "sx1268Main.h"      // SX1268 radio driver (replaces llcc68Main.h)
#include "lowPow.h"
#include "remProto.h"        // RBP: remSendCmd/remSendCmdNew/remHandleRsp (must be after sx1268Main.h)
#include "button.h"
#include "pairRemoteNode.h"  // Starter↔Remote LoRa pairing (must be after sx1268Main.h)

void setup() {
  hwPinInit();

  // ── Anti-glitch settling delay ─────────────────────────────────────────────
  // INPUT_PULLUP lines need ~100 ms to charge through the internal ~50 kΩ after
  // a cold start or rapid power-cycle from a loose battery.  Without this delay
  // the very first digitalRead() can see a floating LOW and falsely trigger the
  // pairing combo.
  delay(300);

  // ── Boot re-pair — deliberate 2-step combo ─────────────────────────────────
  // Step 1: Hold M1_OFF + STA together for ≥ 5 s.
  //         RED LED blinks every 500 ms as visual countdown feedback.
  //         Releasing either button before 5 s aborts silently.
  // Step 2: After the 5 s hold, release both buttons, then press STA once
  //         within 3 s to CONFIRM.  3× WHITE flashes prompt the user.
  //         No confirmation within 3 s → aborts (1× RED flash = cancelled).
  //
  // Two-step requirement prevents a single unintended long-press or a loose
  // battery bounce from accidentally wiping the pairing.
  // enterRemNodePairMode() is called AFTER sx1268Init() so the radio is ready.
  bool bootRepair = false;
  if (digitalRead(M1_OFF_BTN) == LOW && digitalRead(STA_BTN) == LOW) {
    // ── Step 1: 5-second hold with RED blink countdown ──────────────────────
    unsigned long holdStart   = millis();
    unsigned long lastBlinkMs = 0;
    bool          ledOn       = false;
    bool          held5s      = false;

    while (digitalRead(M1_OFF_BTN) == LOW && digitalRead(STA_BTN) == LOW) {
      // Blink RED every 500 ms so the user sees the hold is being registered
      if (millis() - lastBlinkMs >= 500UL) {
        lastBlinkMs = millis();
        if (ledOn) { funcLedReset(); ledOn = false; }
        else        { funcStaLRed();  ledOn = true;  }
      }
      if (millis() - holdStart >= 5000UL) { held5s = true; break; }
    }
    funcLedReset();

    if (held5s) {
      // ── Step 2: 3× rapid WHITE flashes = "confirm with STA press" ─────────
      for (uint8_t i = 0; i < 3; i++) {
        funcStaLWhite(); delay(150); funcLedReset(); delay(150);
      }

      // Wait up to 3 s for a deliberate STA press to confirm
      unsigned long confirmStart = millis();
      while (millis() - confirmStart < 3000UL) {
        if (digitalRead(STA_BTN) == LOW) {
          while (digitalRead(STA_BTN) == LOW) delay(1);  // wait for release
          bootRepair = true;
          break;
        }
      }

      if (!bootRepair) {
        // Timed out without confirmation — 1× RED = aborted
        funcStaLRed(); delay(500); funcLedReset();
      }
    }
    // If Step 1 hold was released early, we fall through with bootRepair = false
  }

  /////////////////////
  if (battCheck()) {
    funcStaLBlue();    // power-on indicator: blue 80 ms
  } else {
    lowBattAlert();
    buzBeep(80); delay(100); buzBeep(80); delay(100); buzBeep(80);  // 3 beeps after 5 blinks
  }
  delay(80);
  funcLedReset();
  ////////////////////
  buzBeep(BUZZ_NOR);
  hwSerialInit();
  getDeviceSerId();   // fills hwSerialKey (chip serial) — the RBP per-remote key source
  sx1268Init();
  lowPowerInit();

  // Trigger re-pair now that radio is initialised.
  // NOTE: do NOT clear the peer serial here — the previous binding must survive a
  // failed/aborted re-pair. The new peer is committed only after the full
  // 0x0D→0x0E→0x0F handshake succeeds (see pairRemoteNode.h). If the re-pair
  // times out, the BEACONING-timeout revert restores the previous peer + channel.
  if (bootRepair) {
    enterRemNodePairMode();   // switches to PAIR channel, starts beaconing
  } else {
    // Hardening: if already paired, switch PAIR→OPER channel HERE (in setup, before
    // loop()/hwbuttonFunc() ever runs) so the radio is deterministically on the
    // operational channel for the very first button press. Previously this switch
    // happened in the first pairRemNodeTick() — which runs AFTER hwbuttonFunc() in the
    // loop — leaving a brief window where an early press could TX on the PAIR channel.
    // Factory-fresh / EEPROM-corrupt devices (no peer serial) are left on the PAIR
    // channel for pairRemNodeTick() to handle (auto-pair or no-peer alert).
    char peerBuf[21];
    readPeerSerial(peerBuf, sizeof(peerBuf));
    if (peerBuf[0] != '\0' && peerBuf[0] != (char)0xFF) {
      switchToOperationalChannel();
      bootPrimePending = true;   // warm the radio before the user's first press (pairRemNodeTick)
      pairBootInitDone = true;   // tell pairRemNodeTick the boot channel switch is already done
      peerSerialCached = 1;      // keep its cache consistent → no duplicate EEPROM read
      buttonEn[0] = ENABLED;     // M1_ON
      buttonEn[1] = ENABLED;     // M1_OFF — always enabled once paired (no status gating)
      buttonEn[2] = ENABLED;     // STA
    }
  }

  if (!wdtEnabled) {
    watchdogInit();
    wdtEnabled = true;
  }
  // Peer serial (Starter ID) is stored via LoRa pairing — no hardcoded ID.
  // pairRemNodeTick() auto-pairs only on factory-fresh EEPROM (sentinel absent).
  // Previously-paired devices with corrupt EEPROM require the boot combo above.
}

void loop() {

  lowPowerPoll();
  hwbuttonFunc();
  ackReception();      // STA retry / timeout handler (defined in button.h)
  sx1268Func();
  pairRemNodeTick();   // pairing state machine; auto-pairs only on factory-fresh device
  // funcLedReset() removed from here — every LED action already resets at its own end.
  // Removing it allows pairRemNodeTick() 4-phase blink to stay visible between loop ticks.

  // Periodic low-battery check every 30 s.
  // WDT is reset before the alert so the ~2.4 s blocking sequence is safe.
  //
  // Guard: skip the check (do NOT advance lastBatCheck) while the radio is actively
  // transmitting. In STATE_TX_SETUP/STATE_TX_WAIT the SX1268 draws ~120 mA at 22 dBm
  // for the ~240 ms TX window; on partially-discharged AAs (elevated ESR) VDD sags,
  // and all 3 reads in battCheck() catch the load-induced sag and fall below
  // BAT_VOL_MIN → a FALSE low-battery alert even though the resting voltage is healthy.
  // By not advancing lastBatCheck, the check retries on the next loop once TX exits
  // (~240 ms later, in STATE_RX_SETUP) and measures VDD at resting load.
  static unsigned long lastBatCheck = 0;
  if (millis() - lastBatCheck >= 30000UL) {
    if (radio_state != STATE_TX_SETUP && radio_state != STATE_TX_WAIT) {
      lastBatCheck = millis();
      if (!battCheck()) {
        if (wdtEnabled) watchdogReset();
        lowBattAlert();
        buzBeep(80); delay(100); buzBeep(80); delay(100); buzBeep(80);  // 3 beeps after 5 blinks
      }
    }
  }

  if (wdtEnabled) {
    watchdogReset();
  }

  delay(5);
}
