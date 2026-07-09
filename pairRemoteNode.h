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
    PAIR_REMNODE_WAIT_TX_DONE,  // got 0x0E, 0x0F TX pending; wait 1 s then await 0x10
    PAIR_REMNODE_WAIT_CONFIRM,  // 0x0F sent; retry it until the Starter's 0x10 confirm
} PairRemNodeState_t;

static PairRemNodeState_t pairRemNodeState       = PAIR_REMNODE_IDLE;
static unsigned long      pairRemNodeBeaconMs    = 0;    // timestamp of last beacon
static unsigned long      pairRemNodeDoneTxMs    = 0;    // timestamp when 0x0F was sent (also 0x0F retry base)
static bool               pairRemNodeDonePending = false;
static uint8_t            pairRemNodeDoneRetries = 0;    // # of 0x0F resends while awaiting 0x10
// One-shot: after a CONFIRMED pairing, send one STATUS so the Starter immediately sees
// authenticated traffic from the new remote (clears its provisional commit) and the
// user gets instant "link works" feedback. Fired from pairRemNodeTick() when radio free.
static bool               pairConfirmStatusPending = false;
// One-shot: after a cold boot reaches the operational channel, emit a silent priming TX
// to warm the radio's PLL/PA. Without it the user's first press IS the radio's first
// cold TX, which is RF-degraded and dropped by the Starter (the retry, on a warm radio,
// works). Fired from pairRemNodeTick() when the radio is free.
static bool               bootPrimePending = false;
static unsigned long      pairRemNodeScanStartMs = 0;    // when the current pairing attempt began
static char               pairRemNodeStarterId[21] = {0}; // staged starter serial (committed only after 0x0F)
// Re-pair scan window: if a previously-paired remote re-pairs but no Starter
// answers within this window, revert to the previous peer (keys + channel intact).
#define PAIR_REMNODE_SCAN_TIMEOUT_MS  60000UL
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
    pairRemNodeScanStartMs = millis();   // start the re-pair scan window
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

    // STAGE ONLY — hold the new starter serial in RAM. It is committed to EEPROM
    // (savePeerSerial / saveOperSyncWord / setPairSentinel) only after the 0x0F
    // DONE TX completes (see PAIR_REMNODE_WAIT_TX_DONE), so an aborted re-pair
    // never overwrites the previous peer keys.
    memcpy(pairRemNodeStarterId, starterId, 21);

    // Store RF params to apply after sending 0x0F
    pairRemNodeAckSf     = buf[21];
    pairRemNodeAckBwCode = buf[22];
    pairRemNodeAckCr     = buf[23];
    pairRemNodeAckPre    = buf[24];
    pairRemNodeAckPwr    = buf[25];

    // Dynamic sync word (bytes 26-27) — stage in RAM only; persisted at commit.
    if (len >= 28) {
        g_operSyncMsb = buf[26];
        g_operSyncLsb = buf[27];
        DEBUG_PRINTN("PairRem: dyn sync word staged");
    }

    // Visual acknowledgement
    funcStaLBlue();
    delay(100);
    funcLedReset();

    // Schedule 0x0F REM_PAIR_DONE TX
    pairRemNodeDonePending = true;
    pairRemNodeState       = PAIR_REMNODE_WAIT_TX_DONE;
}

// ── pairRemNodeCommit() ───────────────────────────────────────────────────────
// COMMIT the staged pairing. Called ONLY when the Starter's 0x10 CONFIRM arrives
// (see dispatchPairPkt) — never on a fire-and-forget timer. This is the single
// commit point; until 0x10 is received the previous peer keys remain in EEPROM.
static void pairRemNodeCommit() {
    watchdogReset();  // ~2.5 s of blocking buzzer + LED + reinit ahead

    // Persist the staged starter serial + sync word and mark "ever paired".
    savePeerSerial(pairRemNodeStarterId);
    saveOperSyncWord(g_operSyncMsb, g_operSyncLsb);
    peerSerialCached = 1;
    remInvalidatePeerCache();   // flush remProto.h RAM cache — next TX re-reads
    setPairSentinel();

    // 1-second beep = paired OK
    buzBeep(1000);
    funcStaLWhite();
    delay(500);
    funcLedReset();

    // Full radio reinit after pairing (see long note previously here): the pairing
    // sequence ran on the pair channel (SF7/0x1234); switchToOperationalChannel()
    // alone does not reapply TCXO/calibration/IRQ-mask/antenna errata/DIO1 re-arm.
    sx1268Init();                   // full hardware reset + recalibration
    switchToOperationalChannel();   // operational SF/BW + dynamic sync word
    pairBootInitDone = true;        // block IDLE from re-switching the channel

    // Motor control buttons: motor OFF at pairing time.
    buttonEn[0] = ENABLED;   // M1_ON
    buttonEn[1] = DISABLED;  // M1_OFF
    buttonEn[2] = ENABLED;   // STA

    pairConfirmStatusPending = true;   // fire one STATUS next tick (reachability proof)
    pairRemNodeState = PAIR_REMNODE_IDLE;
}

// ── pairRemNodeRevert() ───────────────────────────────────────────────────────
// Abandon the staged pairing and restore the PREVIOUS peer + operational channel.
// Nothing was committed to EEPROM, so the old keys are intact — only the RAM sync
// word and radio channel need restoring. Used by the BEACONING scan timeout and the
// WAIT_CONFIRM (no-0x10) timeout.
static void pairRemNodeRevert() {
    watchdogReset();
    loadOperSyncWord(&g_operSyncMsb, &g_operSyncLsb);  // restore previous channel sync
    switchToOperationalChannel();
    buttonEn[0] = ENABLED;    // M1_ON
    buttonEn[1] = DISABLED;   // M1_OFF (motor off)
    buttonEn[2] = ENABLED;    // STA
    pairBootInitDone = true;  // prevent IDLE from re-switching the channel
    // Error feedback: orange + two short beeps = pairing failed, reverted.
    funcStaLOrange(); buzBeep(150); delay(120); buzBeep(150); funcLedReset();
    pairRemNodeState = PAIR_REMNODE_IDLE;
}

// ── pairRemNodeTick() ─────────────────────────────────────────────────────────
// Call every loop(). Must be placed AFTER sx1268Func() in the loop body.
void pairRemNodeTick() {
    unsigned long now = millis();

    // ── Post-pair reachability STATUS (one-shot) ────────────────────────────────
    // After a confirmed pairing, send a single STATUS on the operational channel so
    // the Starter receives authenticated RBP traffic from the new remote and clears
    // its provisional commit. remTxButtonCmd() guards re-entry via msgTxd.
    if (pairConfirmStatusPending && !msgTxd) {
        pairConfirmStatusPending = false;
        remTxButtonCmd(REM_CMD_STATUS);
    }

    // ── Cold-boot radio warm-up (one-shot) ──────────────────────────────────────
    // Emit one silent priming packet on the operational channel to warm the PLL/PA so
    // the user's first real command isn't the cold (RF-degraded, dropped) first TX.
    // Raw send: no msgTxd, no ack wait, no tone, no remSeq bump. byte0=0xAA is not an
    // RBP (0x22) or pairing (0x0D-0x10) type, so the Starter ignores the payload — but
    // it still refreshes the Starter's RX watchdog activity timer.
    if (bootPrimePending) {
        bootPrimePending = false;      // one-shot
        // RE-ENABLED: the DIO2 RF-switch fix reduced but did NOT fully eliminate the cold
        // first-TX RF degradation — the user's first STATUS after a power-cycle was still
        // dropped sporadically (cold PLL/PA settling), recovering only on the ~2.3 s retry.
        // This silent 0xAA ping warms the radio so the first real press is the 2nd (warm) TX.
        // byte0=0xAA is not an RBP (0x22) or pairing (0x0D-0x10) type, so the Starter ignores
        // the payload (logs "RX: ignored pkt[0]=0xAA"); no msgTxd/ack/tone/seq side effects.
        if (!msgTxd) {                 // a real command already in flight needs no priming
            uint8_t ping = 0xAA;
            send_lora_data(&ping, 1);
        }
    }

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
                // Already paired. For a normal paired boot this PAIR→OPER switch is now done
                // in setup() (right after sx1268Init), so pairBootInitDone is already true and
                // this block is skipped. It remains as an idempotent fallback in case setup()
                // did not run it (guarded by pairBootInitDone — never double-switches).
                if (!pairBootInitDone) {
                    pairBootInitDone = true;
                    switchToOperationalChannel();
                    bootPrimePending = true;   // warm the radio before the user's first press
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
            // ── Re-pair abort / revert ─────────────────────────────────────────
            // If this device was already paired (sentinel set) and no Starter
            // answered within the scan window, stop beaconing and restore the
            // PREVIOUS operational channel + peer. Nothing was committed, so the
            // old keys are intact — only the radio channel needs restoring.
            // Factory-fresh devices (no sentinel, first-ever pairing) keep
            // beaconing as before — there is no previous binding to revert to.
            if (isPairSentinelSet() &&
                (now - pairRemNodeScanStartMs >= PAIR_REMNODE_SCAN_TIMEOUT_MS)) {
                DEBUG_PRINTN("PairRem: scan timeout — reverting to previous peer");
                pairRemNodeRevert();
                break;
            }
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

            // Step 2: wait ~1 s for the 0x0F TX to complete, then await the Starter's
            // 0x10 CONFIRM. Do NOT commit here — the commit now happens ONLY when 0x10
            // is received (pairRemNodeCommit, called from dispatchPairPkt). Stay on the
            // pair channel so the 0x10 reply can be received.
            if (now - pairRemNodeDoneTxMs >= 1000UL) {
                pairRemNodeDoneRetries = 0;
                pairRemNodeDoneTxMs    = now;   // reuse as the 0x0F-retry timer base
                pairRemNodeState       = PAIR_REMNODE_WAIT_CONFIRM;
                DEBUG_PRINTN("PairRem: 0x0F sent — awaiting 0x10 confirm");
            }
            break;

        case PAIR_REMNODE_WAIT_CONFIRM:
            // Retransmit 0x0F until the Starter answers with 0x10 (handled async in
            // dispatchPairPkt → pairRemNodeCommit). After PAIR_DONE_MAX_RETRIES with no
            // confirm, revert to the previous peer + channel — the old binding is still
            // intact in EEPROM (nothing was committed), so the existing link keeps working.
            if (now - pairRemNodeDoneTxMs >= PAIR_DONE_RETRY_MS) {
                if (pairRemNodeDoneRetries >= PAIR_DONE_MAX_RETRIES) {
                    DEBUG_PRINTN("PairRem: no 0x10 confirm — reverting to previous peer");
                    pairRemNodeRevert();
                } else {
                    pairRemNodeDoneRetries++;
                    DEBUG_PRINT("PairRem: 0x0F retry ");
                    DEBUG_PRINTN(pairRemNodeDoneRetries);
                    uint8_t pkt[5];
                    pkt[0] = PKT_REM_PAIR_DONE;               // 0x0F
                    memcpy(&pkt[1], hwSerialKey, 4);           // first 4 bytes as echo
                    send_lora_data(pkt, 5);
                    pairRemNodeDoneTxMs = now;
                }
            }
            break;
    }
}

// ── dispatchPairPkt() ─────────────────────────────────────────────────────────
// Called from sx1268Main.h STATE_RX_WAIT when a binary pairing packet arrives.
// On the Remote: 0x0E (REM_PAIR_ACK) starts the DONE handshake; 0x10 (REM_PAIR_CONFIRM)
// is the Starter's final commit confirmation.
void dispatchPairPkt(const uint8_t* buf, uint8_t len) {
    if (buf[0] == PKT_REM_PAIR_ACK) {            // 0x0E
        pairRemNodeHandleAck(buf, len);
    } else if (buf[0] == PKT_REM_PAIR_CONFIRM) { // 0x10 — Starter confirmed our 0x0F
        if (pairRemNodeState == PAIR_REMNODE_WAIT_CONFIRM ||
            pairRemNodeState == PAIR_REMNODE_WAIT_TX_DONE) {
            DEBUG_PRINTN("PairRem: 0x10 confirm received — committing");
            pairRemNodeCommit();
        }
    }
    // All other pair types are silently ignored on the Remote
}

#endif // PAIR_REMOTE_NODE_H
