// ── DEAD FILE — replaced by remProto.h (Remote Binary Protocol, RBP) ─────────
// This file implemented the old AES-CTR text-frame RX/decrypt protocol.
// It is no longer #included from the .ino.  Any accidental re-include is an error.
#error "decrypt.h is superseded by remProto.h — do not include this file"

// Receiver-side: decrypt incoming LoRa buffer

char decBuf[MAX_MESSAGE_LEN + 1];

static inline bool decryptData(const uint8_t* rx_buf, uint8_t rx_len) {

  encapData[0] = '\0';  // always clear — prevents stale command replay (Fix 4)

  if (rx_len == 0 || rx_len > 64) return false;

  char hexBuf[65];
  uint8_t copyLen = (rx_len < sizeof(hexBuf) - 1) ? rx_len : sizeof(hexBuf) - 1;
  memcpy(hexBuf, rx_buf, copyLen);
  hexBuf[copyLen] = '\0';

  // ---- Parse idx + rnd ----
  if (strlen(hexBuf) < 6) {
    //DEBUG_PRINTN(F("RX too short for idx+rnd"));
    return false;
  }

  // Extract idx (2 hex chars → byte)
  // uint8_t idx = (fromHexChar(hexBuf[0]) << 4) | fromHexChar(hexBuf[1]);

  // Extract rnd (next 4 hex chars → uint16_t)
  uint16_t rnd = (fromHexChar(hexBuf[2]) << 12) | (fromHexChar(hexBuf[3]) << 8) | (fromHexChar(hexBuf[4]) << 4) | (fromHexChar(hexBuf[5]));

  // Remaining ciphertext (ASCII hex)
  const char* cipherHex = hexBuf + 6;

  // ---- Build AES key from *this receiver's* chip serial (cached global) ----
  uint8_t key[KEY_LEN] = { 0 };
  const char* last12 = hwSerialKey + 8;  // last 12 chars = 6 bytes
  for (uint8_t i = 0; i < 6; i++) {
    key[(KEY_LEN - 8) + i] = (fromHexChar(last12[i * 2]) << 4) | fromHexChar(last12[i * 2 + 1]);
  }

  // Append rnd into last 2 bytes
  key[KEY_LEN - 2] = rnd >> 8;
  key[KEY_LEN - 1] = rnd & 0xFF;

  //---- Decrypt ----
  if (decryptWithIdx(cipherHex, key, hwSerialKey, decBuf, sizeof(decBuf))) {
    char* start = strchr(decBuf, '[');
    char* end = strchr(decBuf, ']');

    if (start && end && end > start) {
      // Extract text between [ and ]
      size_t len = end - start - 1;  // content length
      if (len < sizeof(encapData)) {
        strncpy(encapData, start + 1, len);
        encapData[len] = '\0';  // null terminate

        //DEBUG_PRINT(F("Extracted: "));
        //DEBUG_PRINTN(encapData);
        return true;
      } else {
        //DEBUG_PRINTN(F("Content too long"));
      }
    } else {
      //DEBUG_PRINTN(F("Invalid message (no brackets)"));
    }
  } else {
    //DEBUG_PRINTN(F("Decrypt failed"));
  }
  return false;
}


void decryptNFunc(const uint8_t* rx_buf, uint8_t rx_len) {
  if (decryptData(rx_buf, rx_len)) {
    rxFunc();   // only called on successful decrypt (Fix 4)
  }
}
