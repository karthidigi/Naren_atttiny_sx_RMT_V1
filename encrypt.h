/////////////////////////////////////////////////////////
#define HEX_BUFFER_LEN (MAX_MESSAGE_LEN * 2 + 8)
static char txBuffer[HEX_BUFFER_LEN + 4];

// Peer serial cache — populated on first TX, invalidated after re-pairing.
static char s_cachedPeerSerial[21] = { '\0' };

// Call after writing a new peer serial to EEPROM (e.g. after pairing).
void invalidatePeerSerialCache() { s_cachedPeerSerial[0] = '\0'; }

static inline void encryptData(const char *msg) {

  // Use cached peer serial; read EEPROM only on first call or after re-pair.
  if (s_cachedPeerSerial[0] == '\0') {
    readPeerSerial(s_cachedPeerSerial, sizeof(s_cachedPeerSerial));
  }
  const char *peerSerialKey = s_cachedPeerSerial;

  // Build AES key from last 12 chars of peer serial (6 bytes via fromHexChar)
  uint8_t key[KEY_LEN] = { 0 };
  const char *last12 = s_cachedPeerSerial + 8;
  for (uint8_t i = 0; i < 6; i++) {
    key[(KEY_LEN - 8) + i] = (fromHexChar(last12[i * 2]) << 4) | fromHexChar(last12[i * 2 + 1]);
  }

  randomSeed(millis());
  uint16_t rnd = random(1, 0xFFFF);
  key[KEY_LEN - 2] = rnd >> 8;
  key[KEY_LEN - 1] = rnd & 0xFF;

  uint8_t idx = random(0, 16);

  char encBuf[MAX_MESSAGE_LEN * 2 + 3];
  if (!encryptWithIdx(msg, key, peerSerialKey, idx, encBuf, sizeof(encBuf))) {
    return;
  }

  // Build TX buffer directly: idx(2 hex) + rnd(4 hex) + encBuf
  // Eliminates snprintf + 96-byte finalBuf stack allocation
  txBuffer[0] = toHexNibble(idx >> 4);
  txBuffer[1] = toHexNibble(idx & 0x0F);
  txBuffer[2] = toHexNibble(rnd >> 12);
  txBuffer[3] = toHexNibble((rnd >> 8) & 0x0F);
  txBuffer[4] = toHexNibble((rnd >> 4) & 0x0F);
  txBuffer[5] = toHexNibble(rnd & 0x0F);
  uint8_t encLen = strlen(encBuf);
  memcpy(txBuffer + 6, encBuf, encLen + 1);
}
