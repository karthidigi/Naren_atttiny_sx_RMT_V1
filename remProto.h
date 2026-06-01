// remProto.h — ATtiny1606 Remote: Lightweight Binary Remote Protocol (RBP)
//
// TX  PKT_REM_CMD (0x22): Remote→Starter  [0x22][cmd][idHi][idLo][seq][mac0][mac1][mac2]  8 bytes
// RX  PKT_REM_RSP (0x23): Starter→Remote  [0x23][status][flags][seqEcho][mac0][mac1][mac2]  7 bytes
//
// MAC = AES-CTR(macKey, IV=0){authBlock}[0..2]  — 3-byte unforgeable tag
// macKey derived from paired starter serial (same derivation both ends after pairing).
//
// Requires (included before this file via .ino):
//   zSettings.h      → PKT_REM_CMD/RSP, REM_CMD_*, REM_STA_* constants
//   aesMain.h        → AES_ctx, AES_init_ctx_iv, AES_CTR_xcrypt_buffer, fromHexChar, hwSerialKey
//   eeprom.h         → readPeerSerial()
//   states.h         → buttonEn[], msgTxd, ENABLED/DISABLED
//   buz.h            → buzBeep()
//   led.h            → funcM1LGreen, funcM1LRed, funcStaLOrange, funcStaLCyan, funcStaLWhite, funcLedReset
//   sx1268Main.h     → send_lora_data()

#ifndef REM_PROTO_H
#define REM_PROTO_H

// ── Rolling TX sequence counter ───────────────────────────────────────────────
// Incremented by remSendCmdNew() (new button press).
// Retries reuse current remSeq (same seq → starter deduplicates, re-sends cached RSP).
static uint8_t remSeq = 0;

// ── Peer serial cache (starter serial stored at pairing) ─────────────────────
static char remPeerSerial[21] = {'\0'};

static void remLoadPeerSerial() {
    if (remPeerSerial[0] == '\0') {
        readPeerSerial(remPeerSerial, sizeof(remPeerSerial));
    }
}

// Call after re-pairing to flush the cache.
void remInvalidatePeerCache() { remPeerSerial[0] = '\0'; }

// ── MAC key from paired starter serial ───────────────────────────────────────
static void remBuildMacKey(uint8_t *key16) {
    remLoadPeerSerial();
    memset(key16, 0, 16);
    const char *last12 = remPeerSerial + 8;          // last 12 hex chars = 6 bytes
    for (uint8_t i = 0; i < 6; i++) {
        key16[10 + i] = (fromHexChar(last12[i * 2]) << 4) | fromHexChar(last12[i * 2 + 1]);
    }
    key16[14] = PKT_REM_CMD;    // 0x22 — protocol version constant
    key16[15] = PKT_REM_RSP;    // 0x23
}

// ── MAC computation ───────────────────────────────────────────────────────────
static void remCalcMac(const uint8_t *key16, const uint8_t *authBlock16, uint8_t *mac3) {
    uint8_t buf[16];
    memcpy(buf, authBlock16, 16);
    uint8_t iv[16] = {0};
    AES_ctx ctx;
    AES_init_ctx_iv(&ctx, key16, iv);
    AES_CTR_xcrypt_buffer(&ctx, buf, 16);
    mac3[0] = buf[0];
    mac3[1] = buf[1];
    mac3[2] = buf[2];
}

// ── Build CMD auth block ──────────────────────────────────────────────────────
// authBlock = [pktType, cmd, idHi, idLo, seq, 0 × 11]
static void remBuildCmdAuthBlock(uint8_t cmd, uint8_t seq, uint8_t *block16) {
    memset(block16, 0, 16);
    block16[0] = PKT_REM_CMD;
    block16[1] = cmd;
    block16[2] = (fromHexChar(hwSerialKey[16]) << 4) | fromHexChar(hwSerialKey[17]);
    block16[3] = (fromHexChar(hwSerialKey[18]) << 4) | fromHexChar(hwSerialKey[19]);
    block16[4] = seq;
}

// ── Send CMD packet ───────────────────────────────────────────────────────────
// Call remSendCmdNew() for a new command (increments seq).
// Call remSendCmd()    for a retry (reuses current seq).
void remSendCmd(uint8_t cmd) {
    uint8_t macKey[16];
    remBuildMacKey(macKey);

    uint8_t authBlock[16];
    remBuildCmdAuthBlock(cmd, remSeq, authBlock);

    uint8_t mac[3];
    remCalcMac(macKey, authBlock, mac);

    uint8_t pkt[8];
    pkt[0] = PKT_REM_CMD;
    pkt[1] = cmd;
    pkt[2] = authBlock[2];      // idHi
    pkt[3] = authBlock[3];      // idLo
    pkt[4] = remSeq;
    pkt[5] = mac[0];
    pkt[6] = mac[1];
    pkt[7] = mac[2];
    send_lora_data(pkt, 8);
}

void remSendCmdNew(uint8_t cmd) {
    remSeq++;
    if (remSeq == 0xFF) remSeq = 0;    // keep 0xFF reserved as "none" on starter
    remSendCmd(cmd);
}

// ── Handle incoming RSP packet ────────────────────────────────────────────────
// [0x23][status][flags][seqEcho][mac0][mac1][mac2]  = 7 bytes
// Called from sx1268Main.h STATE_RX_WAIT when rx_buffer[0] == PKT_REM_RSP.
void remHandleRsp(const uint8_t *buf, uint8_t len) {
    if (len < 7) { return; }

    // 1. MAC verify
    uint8_t macKey[16];
    remBuildMacKey(macKey);
    uint8_t authBlock[16] = {0};
    authBlock[0] = PKT_REM_RSP;
    authBlock[1] = buf[1];      // status
    authBlock[2] = buf[2];      // flags
    authBlock[3] = buf[3];      // seqEcho
    uint8_t mac[3];
    remCalcMac(macKey, authBlock, mac);
    if (buf[4] != mac[0] || buf[5] != mac[1] || buf[6] != mac[2]) {
        DEBUG_PRINTN(F("RBP: RSP MAC fail"));
        return;
    }

    // 2. Seq echo must match what we last sent
    if (buf[3] != remSeq) {
        DEBUG_PRINT(F("RBP: RSP seqEcho mismatch got="));
        DEBUG_PRINT(buf[3]);
        DEBUG_PRINT(F(" exp="));
        DEBUG_PRINTN(remSeq);
        return;
    }

    uint8_t status = buf[1];
    uint8_t flags  = buf[2];

    DEBUG_PRINT(F("RBP: RSP status="));
    DEBUG_PRINT(status);
    DEBUG_PRINT(F(" flags="));
    DEBUG_PRINTN(flags);

    // 3. Clear pending TX state
    msgTxd = 0;

    // 4. Drive LED / buzzer feedback and update button enables
    switch (status) {

        case REM_STA_ON:
            // Motor ON confirmed
            buttonEn[0] = DISABLED;    // M1_ON  (already on)
            buttonEn[1] = ENABLED;     // M1_OFF
            buttonEn[2] = ENABLED;     // STA
            watchdogReset();
            motorOnTone();             // green blink + tone (~2 s, blocks; WDT reset before)
            break;

        case REM_STA_OFF:
            // Motor OFF confirmed
            buttonEn[0] = ENABLED;
            buttonEn[1] = DISABLED;    // M1_OFF (already off)
            buttonEn[2] = ENABLED;
            funcM1LRed();  buzBeep(100); funcLedReset(); delay(100);
            funcM1LRed();  buzBeep(100); funcLedReset();
            break;

        case REM_STA_BLK_BYPASS:
            // Starter blocked: bypass switch active
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
            // Starter blocked: voltage fault
            buttonEn[0] = ENABLED;
            buttonEn[1] = ENABLED;
            buttonEn[2] = ENABLED;
            funcStaLOrange();
            buzBeep(300); delay(100);
            buzBeep(300);
            funcLedReset();
            break;

        case REM_STA_FAULT:
            // Fault / auto-stop
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

    // 5. Update relay state from flags bit 2
    // (flags bit2 = relay/LiREL state — available for future use)
    (void)flags;
}

#endif // REM_PROTO_H
