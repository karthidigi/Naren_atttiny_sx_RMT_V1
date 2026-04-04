#include <EEPROM.h>

#define EEPROM_PEER_SERIAL_ADDR   0x00   // bytes 0–20: peer serial (20 chars + '\0')
#define PEER_SERIAL_LEN           21     // 20 chars + '\0'

// Byte 21: written 0xA5 when pairing completes successfully.
// Distinguishes a factory-fresh device (sentinel absent → auto-pair OK) from a
// device whose peer serial was erased by a brown-out (sentinel present → require
// explicit boot combo, never auto-pair silently).
#define EEPROM_PAIR_SENTINEL_ADDR 0x15   // byte immediately after peer serial
#define EEPROM_PAIR_SENTINEL_VAL  0xA5

static inline bool isPairSentinelSet() {
  return (EEPROM.read(EEPROM_PAIR_SENTINEL_ADDR) == EEPROM_PAIR_SENTINEL_VAL);
}
static inline void setPairSentinel() {
  EEPROM.update(EEPROM_PAIR_SENTINEL_ADDR, EEPROM_PAIR_SENTINEL_VAL);
}

// Save peer serial (20-char hex string + null)
static inline void savePeerSerial(const char *buffer) {
  for (uint8_t i = 0; i < PEER_SERIAL_LEN; i++) {
    char c = buffer[i];
    EEPROM.update(EEPROM_PEER_SERIAL_ADDR + i, c);
    if (c == '\0') break;  // stop early if null terminator found
  }
}

// Read peer serial back into buffer
static inline void readPeerSerial(char *buffer, size_t bufferSize) {
  if (bufferSize < PEER_SERIAL_LEN) {
    buffer[0] = '\0';
    return;
  }

  for (uint8_t i = 0; i < PEER_SERIAL_LEN - 1; i++) {
    buffer[i] = EEPROM.read(EEPROM_PEER_SERIAL_ADDR + i);
    if (buffer[i] == '\0' || buffer[i] == 0xFF) { // handle empty EEPROM
      buffer[i] = '\0';
      break;
    }
  }
  buffer[PEER_SERIAL_LEN - 1] = '\0';
}

// Clear peer serial — forces re-pair on next boot/tick
static inline void clearPeerSerial() {
  EEPROM.update(EEPROM_PEER_SERIAL_ADDR, '\0');
}

// ── Dynamic operational sync word ────────────────────────────────────────────
// Bytes 22–23: sync word bytes received from Starter in 0x0E during pairing.
// Written once on pairing; read on every boot to restore the correct channel.
#define EEPROM_SYNC_MSB_ADDR  0x16
#define EEPROM_SYNC_LSB_ADDR  0x17

static inline void saveOperSyncWord(uint8_t msb, uint8_t lsb) {
  EEPROM.update(EEPROM_SYNC_MSB_ADDR, msb);
  EEPROM.update(EEPROM_SYNC_LSB_ADDR, lsb);
}

// Returns true if valid sync bytes were found; false = use defaults.
static inline bool loadOperSyncWord(uint8_t *msb, uint8_t *lsb) {
  uint8_t m = EEPROM.read(EEPROM_SYNC_MSB_ADDR);
  uint8_t l = EEPROM.read(EEPROM_SYNC_LSB_ADDR);
  if (m == 0xFF || l == 0xFF) return false;  // factory-fresh / not yet set
  *msb = m;
  *lsb = l;
  return true;
}
