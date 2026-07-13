// remProto.h — ATtiny1606 Remote: Plain RBP v2 (fixed-encryption CMD/RSP)
//
// TX  PKT_REM_CMD (0x22): Remote→Starter  [0x22][ E(MAGIC0,MAGIC1,cmd,   0x00 ) ]  5 bytes
// RX  PKT_REM_RSP (0x23): Starter→Remote  [0x23][ E(MAGIC0,MAGIC1,status,light) ]  5 bytes
//
// Cipher: AES-128-CTR, fixed IV, PER-REMOTE key derived from THIS REMOTE'S OWN chip
// serial (hwSerialKey — the same 20 chars sent to the starter in the 0x0D pairing REQ).
// The starter holds every paired remote's serial in its roster and trial-decrypts RX
// packets against each key, so it knows exactly WHICH remote is talking and encrypts
// its RSP with that remote's key. Same 5-byte frame as before — zero added bytes.
// (Old scheme derived from the paired STARTER serial — one shared key for all remotes,
// no sender identity, no real revocation. BREAKING change: starter must run the
// matching roster firmware, and this remote must RE-PAIR after flashing.)
// CTR is symmetric — remCipher4() encrypts and decrypts the 4-byte payload in place.
// NO sequence number, NO MAC (vs v1 → archive/remProto_rbp_v1.h):
// the 2-byte MAGIC inside the encrypted payload is the integrity/identity check, and the
// starter time-window-dedups retransmits. The remote sends the same bytes on a retry
// (deterministic ciphertext) and accepts ANY valid RSP (no seqEcho to match).
//
// Requires (included before this file via .ino):
//   zSettings.h      → PKT_REM_CMD/RSP, REM_CMD_*, REM_STA_*, RBP_MAGIC0/1
//   aesMain.h        → AES_ctx, AES_init_ctx_iv, AES_CTR_xcrypt_buffer, fromHexChar,
//                      hwSerialKey (own chip serial, filled by getDeviceSerId at boot)
//   states.h         → buttonEn[], msgTxd, ENABLED/DISABLED
//   buz.h / led.h    → buzBeep(), funcM1Yellow/funcM1LRed/funcStaL*/funcLedReset, motorOnTone()
//   sx1268Main.h     → send_lora_data()

#ifndef REM_PROTO_H
#define REM_PROTO_H

// Most recent command type sent. Lets remHandleRsp give light-switch feedback for RELAY
// commands vs motor feedback for M1 commands.
static uint8_t remLastSentCmd = 0;

// Kept as a no-op for the pairRemoteNode.h call site: the RBP key now derives from
// THIS remote's own serial (constant for the life of the chip), so there is no
// peer-serial cache left to invalidate after a re-pair.
void remInvalidatePeerCache() {}

// ── Per-remote cipher key from THIS remote's own chip serial ──────────────────
// All 20 hex chars (10 bytes) of the serial go into key bytes 0..9 for maximum
// per-unit entropy. Bytes 14/15 are the protocol-version salt. MUST match the
// starter's remBuildKeyFor() in Naren_FSMC_AVRDB_RMT_V1 remProto.h exactly.
static void remBuildKey(uint8_t *key16) {
    memset(key16, 0, 16);
    for (uint8_t i = 0; i < 10; i++) {
        key16[i] = (fromHexChar(hwSerialKey[i * 2]) << 4) | fromHexChar(hwSerialKey[i * 2 + 1]);
    }
    key16[14] = PKT_REM_CMD;    // 0x22 — protocol-version salt
    key16[15] = PKT_REM_RSP;    // 0x23
}

// ── AES-CTR encrypt/decrypt of the 4-byte payload, in place (symmetric, fixed IV) ──
static void remCipher4(uint8_t *buf4) {
    uint8_t key16[16];
    remBuildKey(key16);
    uint8_t blk[16];
    memset(blk, 0, 16);
    memcpy(blk, buf4, 4);
    uint8_t iv[16] = {0};
    AES_ctx ctx;
    AES_init_ctx_iv(&ctx, key16, iv);
    AES_CTR_xcrypt_buffer(&ctx, blk, 16);
    memcpy(buf4, blk, 4);
}

// ── Send CMD packet (5 bytes) ─────────────────────────────────────────────────
// No seq in v2: a new command and a retry are byte-identical (deterministic ciphertext);
// the starter time-window-dedups retries. remSendCmdNew() is kept as an alias so the
// button/retry call sites are unchanged.
void remSendCmd(uint8_t cmd) {
    remLastSentCmd = cmd;
    uint8_t pl[4] = { RBP_MAGIC0, RBP_MAGIC1, cmd, 0x00 };
    remCipher4(pl);
    uint8_t pkt[5] = { PKT_REM_CMD, pl[0], pl[1], pl[2], pl[3] };
    send_lora_data(pkt, 5);
}

void remSendCmdNew(uint8_t cmd) { remSendCmd(cmd); }

// ── Handle incoming RSP packet (5 bytes) ──────────────────────────────────────
// [0x23][ E(MAGIC0, MAGIC1, status, light) ]
// Called from sx1268Main.h STATE_RX_WAIT when rx_buffer[0] == PKT_REM_RSP.
void remHandleRsp(const uint8_t *buf, uint8_t len) {
    if (len < 5) { return; }

    // Decrypt payload + verify MAGIC (integrity; foreign/noise → wrong magic → drop).
    uint8_t pl[4] = { buf[1], buf[2], buf[3], buf[4] };
    remCipher4(pl);
    if (pl[0] != RBP_MAGIC0 || pl[1] != RBP_MAGIC1) {
        DEBUG_PRINTN(F("RBP: RSP bad magic"));
        return;
    }

    uint8_t status = pl[2];
    uint8_t light  = pl[3];   // dedicated light-switch field (1=ON, 0=OFF)

    DEBUG_PRINT(F("RBP: RSP status="));
    DEBUG_PRINT(status);
    DEBUG_PRINT(F(" light="));
    DEBUG_PRINTN(light);

    // ── OFF-pre-empt stale-ACK guard ──────────────────────────────────────────
    // If OFF pre-empted a pending ON, the starter's deferred ON-ack can cross our
    // M1_OFF on air. Accepting that "ON" RSP as the OFF's answer would clear
    // msgTxd and silently kill the OFF retry. So while an M1_OFF is in flight, a
    // RSP still reporting ON is treated as stale — ignored; the retry continues
    // until a non-ON RSP (or the no-network fallback re-enables everything).
    if (msgTxd && lastTxCmdCode == REM_CMD_M1_OFF && status == REM_STA_ON) {
        DEBUG_PRINTN(F("RBP: stale ON-ack during OFF — keep waiting"));
        return;
    }

    // Any valid RSP clears the pending TX (no seqEcho in v2).
    msgTxd = 0;

    // Always-enabled motor buttons: enables no longer track the starter's motor
    // status — every valid RSP just re-opens all three keys. The starter is
    // idempotent + deduped, so a redundant ON/OFF is harmless (answered from
    // cache, no re-actuation).
    buttonEn[0] = ENABLED;
    buttonEn[1] = ENABLED;
    buttonEn[2] = ENABLED;

    // Light-switch RSP — RELAY command → light feedback (only when ON, per spec).
    if (remLastSentCmd == REM_CMD_RELAY_ON || remLastSentCmd == REM_CMD_RELAY_OFF) {
        if (light != 0) {
            funcM1Yellow(); buzBeep(100); funcLedReset();
        }
        return;
    }

    // Motor RSP — LED/buzzer feedback per status.
    switch (status) {

        case REM_STA_ON:
            watchdogReset();
            motorOnTone();             // green blink + tone (~0.5 s, blocks; WDT reset before)
            break;

        case REM_STA_OFF:
            funcM1LRed();  buzBeep(100); funcLedReset(); delay(100);
            funcM1LRed();  buzBeep(100); funcLedReset();
            break;

        case REM_STA_BLK_BYPASS:
            funcStaLCyan();
            buzBeep(100); delay(80);
            buzBeep(100); delay(80);
            buzBeep(100);
            funcLedReset();
            break;

        case REM_STA_BLK_VFAULT:
            funcStaLOrange();
            buzBeep(300); delay(100);
            buzBeep(300);
            funcLedReset();
            break;

        case REM_STA_FAULT:
            funcStaLOrange(); buzBeep(100); funcLedReset(); delay(100);
            funcStaLOrange(); buzBeep(100); funcLedReset();
            break;

        default:
            break;
    }
}

#endif // REM_PROTO_H
