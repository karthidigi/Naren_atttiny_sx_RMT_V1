// remProto.h — ATtiny1606 Remote: Plain RBP v2 (fixed-encryption CMD/RSP)
//
// TX  PKT_REM_CMD (0x22): Remote→Starter  [0x22][ E(MAGIC0,MAGIC1,cmd,   0x00 ) ]  5 bytes
// RX  PKT_REM_RSP (0x23): Starter→Remote  [0x23][ E(MAGIC0,MAGIC1,status,light) ]  5 bytes
//
// Cipher: AES-128-CTR, fixed IV, PER-UNIT key derived from the paired starter serial
// (same derivation both ends). CTR is symmetric — remCipher4() encrypts and decrypts the
// 4-byte payload in place. NO sequence number, NO MAC (vs v1 → archive/remProto_rbp_v1.h):
// the 2-byte MAGIC inside the encrypted payload is the integrity/identity check, and the
// starter time-window-dedups retransmits. The remote sends the same bytes on a retry
// (deterministic ciphertext) and accepts ANY valid RSP (no seqEcho to match).
//
// Requires (included before this file via .ino):
//   zSettings.h      → PKT_REM_CMD/RSP, REM_CMD_*, REM_STA_*, RBP_MAGIC0/1
//   aesMain.h        → AES_ctx, AES_init_ctx_iv, AES_CTR_xcrypt_buffer, fromHexChar
//   eeprom.h         → readPeerSerial()
//   states.h         → buttonEn[], msgTxd, ENABLED/DISABLED
//   buz.h / led.h    → buzBeep(), funcM1Yellow/funcM1LRed/funcStaL*/funcLedReset, motorOnTone()
//   sx1268Main.h     → send_lora_data()

#ifndef REM_PROTO_H
#define REM_PROTO_H

// Most recent command type sent. Lets remHandleRsp give light-switch feedback for RELAY
// commands vs motor feedback for M1 commands.
static uint8_t remLastSentCmd = 0;

// ── Peer serial cache (starter serial stored at pairing) ─────────────────────
static char remPeerSerial[21] = {'\0'};

static void remLoadPeerSerial() {
    if (remPeerSerial[0] == '\0') {
        readPeerSerial(remPeerSerial, sizeof(remPeerSerial));
    }
}

// Call after re-pairing to flush the cache.
void remInvalidatePeerCache() { remPeerSerial[0] = '\0'; }

// ── Per-unit cipher key from the paired starter serial ───────────────────────
static void remBuildKey(uint8_t *key16) {
    remLoadPeerSerial();
    memset(key16, 0, 16);
    const char *last12 = remPeerSerial + 8;          // last 12 hex chars = 6 bytes
    for (uint8_t i = 0; i < 6; i++) {
        key16[10 + i] = (fromHexChar(last12[i * 2]) << 4) | fromHexChar(last12[i * 2 + 1]);
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

    // Any valid RSP clears the pending TX (no seqEcho in v2).
    msgTxd = 0;

    // Light-switch RSP — RELAY command → light feedback (only when ON, per spec).
    // Button enables still track motor state from `status`.
    if (remLastSentCmd == REM_CMD_RELAY_ON || remLastSentCmd == REM_CMD_RELAY_OFF) {
        bool lightOn = (light != 0);
        if (status == REM_STA_ON) { buttonEn[0] = DISABLED; buttonEn[1] = ENABLED; }
        else                      { buttonEn[0] = ENABLED;  buttonEn[1] = DISABLED; }
        buttonEn[2] = ENABLED;
        if (lightOn) {
            funcM1Yellow(); buzBeep(100); funcLedReset();
        }
        return;
    }

    // Motor RSP — drive LED/buzzer feedback and update button enables.
    switch (status) {

        case REM_STA_ON:
            buttonEn[0] = DISABLED;    // M1_ON  (already on)
            buttonEn[1] = ENABLED;     // M1_OFF
            buttonEn[2] = ENABLED;     // STA
            watchdogReset();
            motorOnTone();             // green blink + tone (~0.5 s, blocks; WDT reset before)
            break;

        case REM_STA_OFF:
            buttonEn[0] = ENABLED;
            buttonEn[1] = DISABLED;    // M1_OFF (already off)
            buttonEn[2] = ENABLED;
            funcM1LRed();  buzBeep(100); funcLedReset(); delay(100);
            funcM1LRed();  buzBeep(100); funcLedReset();
            break;

        case REM_STA_BLK_BYPASS:
            buttonEn[0] = ENABLED;
            buttonEn[1] = ENABLED;
            buttonEn[2] = ENABLED;
            funcStaLCyan();
            buzBeep(100); delay(80);
            buzBeep(100); delay(80);
            buzBeep(100);
            funcLedReset();
            break;

        case REM_STA_BLK_VFAULT:
            buttonEn[0] = ENABLED;
            buttonEn[1] = ENABLED;
            buttonEn[2] = ENABLED;
            funcStaLOrange();
            buzBeep(300); delay(100);
            buzBeep(300);
            funcLedReset();
            break;

        case REM_STA_FAULT:
            buttonEn[0] = ENABLED;
            buttonEn[1] = DISABLED;
            buttonEn[2] = ENABLED;
            funcStaLOrange(); buzBeep(100); funcLedReset(); delay(100);
            funcStaLOrange(); buzBeep(100); funcLedReset();
            break;

        default:
            buttonEn[0] = ENABLED;
            buttonEn[1] = ENABLED;
            buttonEn[2] = ENABLED;
            break;
    }
}

#endif // REM_PROTO_H
