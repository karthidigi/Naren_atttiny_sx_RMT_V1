// pairRemoteNode.h — Remote (ATtiny1606) side: Starter ↔ Remote pairing
//
// FLOW:
//   pairRemNodeTick()        → auto-enter if EEPROM peer serial is empty
//   enterRemNodePairMode()   → switchToPairChannel() → PAIR_REMNODE_BEACONING
//   BEACONING                → TX [0x0D][hwSerialKey:20] every 2 s
//   pairRemNodeHandleAck()   → on 0x0E: store starter ID, schedule 0x0F TX
//   WAIT_TX_DONE (tick)      → send 0x0F, wait 1 s, enable buttons, switchToOperationalChannel()
//
// ALSO defines dispatchPairPkt() — binary-packet entry point on the Remote.
// Only handles 0x0E (REM_PAIR_ACK from Starter); ignores all other pair types.

#ifndef PAIR_REMOTE_NODE_H
#define PAIR_REMOTE_NODE_H

#include "zSettings.h"   // PAIR_*/OPER_*/PKT_* constants, timing defines
#include "states.h"      // buttonEn[], ENABLED/DISABLED

// ── State enum ────────────────────────────────────────────────────────────────
typedef enum {
    PAIR_REMNODE_IDLE = 0,
    PAIR_REMNODE_BEACONING,     // TX [0x0D] every 2 s, waiting for 0x0E
    PAIR_REMNODE_WAIT_TX_DONE,  // got 0x0E, 0x0F TX pending; wait 1 s then switch channel
} PairRemNodeState_t;

static PairRemNodeState_t pairRemNodeState       = PAIR_REMNODE_IDLE;
static unsigned long      pairRemNodeBeaconMs    = 0;    // timestamp of last beacon
static unsigned long      pairRemNodeDoneTxMs    = 0;    // timestamp when 0x0F was sent
static bool               pairRemNodeDonePending = false;
// Cached peer-serial state — updated once on boot and after every pairing.
// Avoids reading EEPROM (20 bytes) on every loop() iteration.
// -1 = unchecked, 0 = no peer stored, 1 = peer stored
static int8_t             peerSerialCached       = -1;
// One-shot: switch radio from PAIR→OPER channel on first IDLE tick after boot.
static bool               pairBootInitDone       = false;

// ── Pairing-active LED blink (4-phase: blue ON → OFF → red ON → OFF) ──────────
// Phase 0 = blue ON (200 ms), 1 = off (300 ms), 2 = red ON (200 ms), 3 = off (300 ms)
static unsigned long pairLedPhaseMs = 0;
static uint8_t       pairLedPhase   = 0;

// Operational RF params received in 0x0E (used to apply correct channel)
static uint8_t pairRemNodeAckSf     = OPER_SF;
static uint8_t pairRemNodeAckBwCode = 1;
static uint8_t pairRemNodeAckCr     = OPER_CR;
static uint8_t pairRemNodeAckPre    = OPER_PREAMBLE;
static uint8_t pairRemNodeAckPwr    = OPER_TX_POWER;

// ── enterRemNodePairMode() ────────────────────────────────────────────────────
// Called automatically when EEPROM peer serial is empty, or after clearPeerSerial().
void enterRemNodePairMode() {
    switchToPairChannel();
    pairRemNodeBeaconMs    = 0;     // trigger immediate first beacon
    pairRemNodeDonePending = false;
    pairRemNodeState       = PAIR_REMNODE_BEACONING;
    // Blue LED blink: indicate pairing mode to user
    funcStaLBlue();
    delay(300);
    funcLedReset();
}

// ── pairRemNodeHandleAck() ────────────────────────────────────────────────────
// Called from dispatchPairPkt() when rx_buffer[0] == PKT_REM_PAIR_ACK (0x0E).
// Packet: [0x0E][starter_id:20][sf:1][bwCode:1][cr:1][preamble:1][txPow:1][syncMsb:1][syncLsb:1]
//         = 28 bytes (bytes 26-27 are dynamic sync word, absent in legacy 26-byte packets)
void pairRemNodeHandleAck(const uint8_t* buf, uint8_t len) {
    if (len < 26) return;
    if (pairRemNodeState != PAIR_REMNODE_BEACONING) return;

    // Extract starter ID (first 20 bytes after type byte)
    char starterId[21];
    memcpy(starterId, &buf[1], 20);
    starterId[20] = '\0';

    // Save starter ID as peer serial in EEPROM; update cache
    savePeerSerial(starterId);
    peerSerialCached = 1;             // cache: peer now stored
    remInvalidatePeerCache();         // flush remProto.h RAM cache — next TX re-reads

    // Store RF params to apply after sending 0x0F
    pairRemNodeAckSf     = buf[21];
    pairRemNodeAckBwCode = buf[22];
    pairRemNodeAckCr     = buf[23];
    pairRemNodeAckPre    = buf[24];
    pairRemNodeAckPwr    = buf[25];

    // Dynamic sync word (bytes 26-27, present in updated starters)
    if (len >= 28) {
        g_operSyncMsb = buf[26];
        g_operSyncLsb = buf[27];
        saveOperSyncWord(g_operSyncMsb, g_operSyncLsb);
        DEBUG_PRINTN("PairRem: dyn sync word stored");
    }

    // Visual acknowledgement
    funcStaLBlue();
    delay(100);
    funcLedReset();

    // Schedule 0x0F REM_PAIR_DONE TX
    pairRemNodeDonePending = true;
    pairRemNodeState       = PAIR_REMNODE_WAIT_TX_DONE;
}

// ── pairRemNodeTick() ─────────────────────────────────────────────────────────
// Call every loop(). Must be placed AFTER sx1268Func() in the loop body.
void pairRemNodeTick() {
    unsigned long now = millis();

    // ── Pairing-active LED: 4-phase blink (self-managed, no loop funcLedReset needed)
    //    Phase 0: blue ON  200 ms
    //    Phase 1: all OFF  300 ms
    //    Phase 2: red  ON  200 ms
    //    Phase 3: all OFF  300 ms
    if (pairRemNodeState != PAIR_REMNODE_IDLE) {
        lowPowerKick();   // prevent sleep entry while pairing is in progress
        unsigned long phaseDur = (pairLedPhase & 1) ? 300UL : 200UL;
        if (now - pairLedPhaseMs >= phaseDur) {
            pairLedPhaseMs = now;
            pairLedPhase   = (pairLedPhase + 1) & 3;
            switch (pairLedPhase) {
                case 0: funcStaLBlue(); break;
                case 1: funcLedReset(); break;
                case 2: funcStaLRed();  break;
                default: funcLedReset(); break;   // phase 3
            }
        }
    }

    switch (pairRemNodeState) {

        case PAIR_REMNODE_IDLE: {
            // Read EEPROM only once (cache = -1 on first call after boot or after
            // clearPeerSerial()). After that the cached value is reused so we don't
            // burn 20 EEPROM reads every 5 ms loop iteration.
            if (peerSerialCached < 0) {
                char peerBuf[21];
                readPeerSerial(peerBuf, sizeof(peerBuf));
                peerSerialCached = (peerBuf[0] != '\0' && peerBuf[0] != (char)0xFF) ? 1 : 0;
            }

            if (peerSerialCached == 0) {
                if (!isPairSentinelSet()) {
                    // Sentinel absent → device has never completed a successful pairing
                    // (factory-fresh EEPROM all 0xFF/0x00). Auto-enter pairing so a
                    // brand-new device pairs without any extra steps.
                    peerSerialCached = -1;
                    enterRemNodePairMode();
                } else {
                    // Sentinel present but peer serial missing → EEPROM corruption
                    // (e.g. brown-out mid-write on a previously-paired device).
                    // DO NOT auto-pair: a loose battery or unexpected reset must never
                    // silently put a field-deployed remote into pairing mode.
                    // Alert every 10 s so the installer knows a re-pair boot combo
                    // (M1_OFF + STA ≥ 5 s → confirm STA) is needed.
                    static unsigned long noPeerAlertMs = 0;
                    static bool          noPeerAlertOnce = false;
                    if (!noPeerAlertOnce || (millis() - noPeerAlertMs >= 10000UL)) {
                        noPeerAlertOnce = true;
                        noPeerAlertMs   = millis();
                        for (uint8_t i = 0; i < 3; i++) {
                            funcStaLRed(); buzBeep(100); funcLedReset(); delay(150);
                        }
                    }
                    // Buttons stay DISABLED — device cannot operate without a valid peer.
                }
            } else {
                // Already paired. sx1268Init() always starts on PAIR channel.
                // Switch to operational channel once so communication works on
                // cold power-on without needing a re-pair.
                if (!pairBootInitDone) {
                    pairBootInitDone = true;
                    switchToOperationalChannel();
                    // Enable buttons (motors are off at boot — ON buttons active)
                    buttonEn[0] = ENABLED;   // M1_ON
                    buttonEn[1] = DISABLED;  // M1_OFF (motor off — can't turn off again)
                    buttonEn[2] = ENABLED;   // STA
                }
                // else: already on operational channel — nothing to do
            }
            break;
        }

        case PAIR_REMNODE_BEACONING:
            // Beacon every PAIR_BEACON_INTERVAL_MS
            if (now - pairRemNodeBeaconMs >= PAIR_BEACON_INTERVAL_MS) {
                uint8_t pkt[21];
                pkt[0] = PKT_REM_PAIR_REQ;                // 0x0D
                memcpy(&pkt[1], hwSerialKey, 20);          // remote chip serial
                send_lora_data(pkt, 21);
                pairRemNodeBeaconMs = now;
            }
            break;

        case PAIR_REMNODE_WAIT_TX_DONE:
            // Step 1: send 0x0F REM_PAIR_DONE on pair channel
            if (pairRemNodeDonePending) {
                uint8_t pkt[5];
                pkt[0] = PKT_REM_PAIR_DONE;               // 0x0F
                memcpy(&pkt[1], hwSerialKey, 4);           // first 4 bytes as echo
                send_lora_data(pkt, 5);
                pairRemNodeDonePending = false;
                pairRemNodeDoneTxMs    = now;
                break;
            }

            // Step 2: wait ~1 s for TX to complete, then switch to operational channel
            if (now - pairRemNodeDoneTxMs >= 1000UL) {
                watchdogReset();  // ~2.5 s of blocking buzzer + LED + reinit ahead

                // Mark EEPROM as "ever successfully paired".
                // From this point on, if the peer serial is ever found missing on
                // boot (brown-out corruption), the IDLE handler will block auto-pair
                // and require the deliberate boot combo instead.
                setPairSentinel();

                // 1-second beep = paired OK
                buzBeep(1000);
                funcStaLWhite();
                delay(500);
                funcLedReset();

                // ── Full radio reinit after pairing ───────────────────────────────
                // The pairing sequence runs multiple TX/RX cycles on the pair channel
                // (SF7/0x1234).  switchToOperationalChannel() alone does NOT reapply:
                //   • TCXO configuration
                //   • Full RF calibration (image rejection, etc.)
                //   • setDioIrqParams  (IRQ mask that makes DIO1 fire on RX_DONE)
                //   • fixResistanceAntenna errata workaround
                //   • DIO1 pin interrupt re-arm (PORTA.PIN7CTRL)
                // Without these, RX sensitivity degrades after pairing and the first
                // S? response from the Starter is silently missed → 4× retries.
                // enterSleep() already does sx1268Init()+switchToOperationalChannel()
                // on every wake-up; replicate that here for the same guarantee.
                sx1268Init();                   // full hardware reset + recalibration
                switchToOperationalChannel();   // SF11/BW125/sync=0x3444

                // ── Prevent duplicate switchToOperationalChannel() in IDLE ────────
                // pairBootInitDone is normally set in the IDLE branch on a cold boot.
                // After pairing it was never set, so the very next IDLE tick would
                // call switchToOperationalChannel() again, issuing SX1268_setStandby()
                // just as the first RX window opens — aborting reception of the first
                // S? ACK.  Set it here to block that second call.
                pairBootInitDone = true;

                // Enable motor control buttons immediately after pairing.
                // states.h boots with all action buttons DISABLED; they would
                // normally only be enabled after receiving an ACK from the Starter.
                // Enable them now so the user can send commands right away.
                // Default state: motor OFF (ON button enabled, OFF button disabled).
                buttonEn[0] = ENABLED;   // M1_ON
                buttonEn[1] = DISABLED;  // M1_OFF  (motor off — can't turn off again)
                buttonEn[2] = ENABLED;   // STA

                pairRemNodeState = PAIR_REMNODE_IDLE;
            }
            break;
    }
}

// ── dispatchPairPkt() ─────────────────────────────────────────────────────────
// Called from sx1268Main.h STATE_RX_WAIT when a binary pairing packet arrives.
// On the Remote, only 0x0E (REM_PAIR_ACK from Starter) is handled.
void dispatchPairPkt(const uint8_t* buf, uint8_t len) {
    if (buf[0] == PKT_REM_PAIR_ACK) {   // 0x0E
        pairRemNodeHandleAck(buf, len);
    }
    // All other pair types are silently ignored on the Remote
}

#endif // PAIR_REMOTE_NODE_H
