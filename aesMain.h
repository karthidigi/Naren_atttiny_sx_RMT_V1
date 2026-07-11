#include "src/aes.h"

#define MAX_MESSAGE_LEN 32
#define KEY_LEN 16
#define BLOCK_SIZE 16

char hwSerialKey[21];  // for encryption purpose
// ------------------------------------------------------
// Convert a nibble (0–15) to hex char
// ------------------------------------------------------
static inline char toHexNibble(uint8_t v) {
  return (v < 10) ? ('0' + v) : ('A' + v - 10);
}

// ------------------------------------------------------
// Hex char → nibble (used by remProto.h remBuildKey)
// ------------------------------------------------------
static inline uint8_t fromHexChar(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return 0;  // fallback
}

// ------------------------------------------------------
// Read chip serial into hex string (20 chars + null)
// ------------------------------------------------------
static inline void getChipSerial(char *out, size_t outLen) {
  if (outLen < 21) return;
  for (uint8_t i = 0; i < 10; i++) {
    uint8_t serByte = *((uint8_t *)&SIGROW.SERNUM0 + i);
    out[i * 2] = toHexNibble(serByte >> 4);
    out[i * 2 + 1] = toHexNibble(serByte & 0x0F);
  }
  out[20] = '\0';
}

// ── Legacy AES text-frame self-test removed (Tier-1 flash reduction) ──────────
// aesInit() plus the old hex text-frame codec (encryptWithIdx/decryptWithIdx/
// getNonce/deriveSessionSeed/bytesToHex/hexToBytes) were the last remnants of the
// pre-RBP protocol — the boot self-test discarded its result and was the only
// caller keeping ~926 B of dead code linked. The live RBP path (remProto.h →
// remCipher4 → AES_CTR_xcrypt_buffer, keyed by getChipSerial/fromHexChar above)
// is unchanged. Old text-frame codec preserved in archive/ if ever needed.
