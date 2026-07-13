#pragma once
/////////////////////////////////////////////////////
#define FIRMWARE_VERSION        "ATtiny1606"
#define HARDWARE_VERSION        "1.1.1"

/////////////////////////////////////////////////////
#define SERIAL_DEBUG   // uncomment for debug output via Serial (UART2)
#define SERIAL_BAUD     115200
#define SERIAL_TIMEOUT  100


/////////////////////////////////////////////////////
#define WATCHDOG 1

/////////////////////////////////////////////////////
#define LP_TIMEOUT_MS (30UL * 1000UL) // 1 minutes

/////////////////////////////////////////////////////
#define BUZZ_NOR 120
#define BUZZ_OFF 600


//////////////////////////////////////////////////////
#define BAT_VOL_MIN 2200


//////////////////////////////////////////////////////
#define MOT_CONT_ID "42407197000090220136"

// ── LoRa Pairing channel (SF7, distinct sync word) ──
// MUST match Starter zSettings.h
#define PAIR_SF           7
#define PAIR_BW           SX126X_LORA_BW_125
#define PAIR_CR           SX126X_LORA_CR_4_5
#define PAIR_PREAMBLE     8
#define PAIR_SYNC_MSB     0x12
#define PAIR_SYNC_LSB     0x34
#define PAIR_TX_POWER     14
// LDRO: SF7+BW125 → symbol time 1.0 ms < 16 ms → OFF (literal 0)

// ── Operational LoRa profile (compile-time; pick EXACTLY ONE) ──────────────
// MUST be identical on remote AND starter — a mismatch prevents any communication.
//
//  Profile             SF  BW     CR    LDRO  ToA/pkt   Use-case
//  SF11_BW125 (★)      11  125kHz  4/8   ON   ~1.7 s    Recommended (long range)
//  SF12_BW125          12  125kHz  4/8   ON   ~3.4 s    Extreme range (+3 dB vs SF11)
//  SF10_BW250          10  250kHz  4/5   OFF  ~0.26 s   Fast/short-range baseline
//
// LDRO: required ON when symbol time > 16 ms (SF11/SF12 at BW125).
// #define LORA_PROFILE_SF11_BW125  // long range + reasonable speed
#define LORA_PROFILE_SF12_BW125     // ← ACTIVE — MUST match the starter (it runs SF12)
// #define LORA_PROFILE_SF10_BW250  // fast/short-range baseline

#if defined(LORA_PROFILE_SF11_BW125)
  #define OPER_SF        11
  #define OPER_BW        SX126X_LORA_BW_125
  #define OPER_BW_CODE   1
  #define OPER_CR        SX126X_LORA_CR_4_8   // max FEC (+~3 dB link margin vs CR4/5) — MUST match starter
  #define OPER_LDRO      1    // SF11+BW125 symbol time 16.4 ms → LDRO ON
  #define OPER_TOA_MS    720UL           // SF11/BW125/CR4/8/pre12/PL5/explicit ≈ 660 ms + margin
#elif defined(LORA_PROFILE_SF12_BW125)
  #define OPER_SF        12
  #define OPER_BW        SX126X_LORA_BW_125
  #define OPER_BW_CODE   1
  #define OPER_CR        SX126X_LORA_CR_4_8   // max FEC for extreme range
  #define OPER_LDRO      1    // SF12+BW125 symbol time 32.8 ms → LDRO ON
  #define OPER_TOA_MS    1150UL          // SF12/BW125/CR4/8/pre12/PL5/explicit ≈ 1057 ms + margin
#elif defined(LORA_PROFILE_SF10_BW250)
  #define OPER_SF        10
  #define OPER_BW        SX126X_LORA_BW_250
  #define OPER_BW_CODE   2
  #define OPER_CR        SX126X_LORA_CR_4_5   // original fast profile
  #define OPER_LDRO      0    // SF10+BW250 symbol time 4.1 ms → LDRO OFF
  #define OPER_TOA_MS    200UL           // SF10/BW250/CR4/5/pre12/PL5/explicit ≈ 140 ms + margin
#else
  #error "zSettings.h: define LORA_PROFILE_SF11_BW125, LORA_PROFILE_SF12_BW125, or LORA_PROFILE_SF10_BW250"
#endif

// OPER_CR is defined per-profile above.
#define OPER_PREAMBLE     12   // MUST match starter (tried 24 for low-SNR margin; reverted to 12 — worked better)

// ── OPER channel header mode ───────────────────────────────────────────────
// All RBP v2 frames are exactly 5 bytes. Implicit header removes the 3-byte
// explicit header and its per-packet header-CRC (the source of "RX Header Error").
// Both ends MUST use the same setting — change in both zSettings.h files together.
//
// Select with this LOCAL 0/1 flag — do NOT branch by comparing OPER_HEADER_MODE
// against the driver's SX126X_LORA_PKT_* enums in #if. This file is parsed BEFORE
// src/sx126x.h is included (zSettings.h at .ino top; driver at sx1268Main.h),
// so those enums are still UNDEFINED here and an #if comparison silently evaluates
// (0 == 0) → always picks "implicit", which masked the HEADER_ERR IRQ out even in
// explicit mode (so "RX Header Error" stopped printing). The defines below expand only
// at point-of-use in sx1268Main.h, where the driver header is already included.
#define OPER_USE_IMPLICIT_HDR  0   // 0 = explicit header, 1 = implicit (fixed 5-byte)
#if OPER_USE_IMPLICIT_HDR
  #define OPER_HEADER_MODE  SX126X_LORA_PKT_IMPLICIT
  #define OPER_HDR_ERR_IRQ  0                      // no on-air header → HEADER_ERR can never fire
#else
  #define OPER_HEADER_MODE  SX126X_LORA_PKT_EXPLICIT
  #define OPER_HDR_ERR_IRQ  SX126X_IRQ_HEADER_ERROR  // enable header-error IRQ + logging
#endif
#define OPER_PAYLOAD_LEN  5   // fixed RBP frame size — CMD and RSP are both 5 bytes

// ── RX gain (reg 0x08AC) ────────────────────────────────────────────────────
// 0x94 = power-saving/default, 0x96 = boosted (+~3 dB sensitivity). Boosted helps
// long range but can OVERLOAD the front end at near-field (strong RSSI), which is a
// known cause of RX Header Errors at ~−30..−40 dBm. Default to power-saving to test
// whether the near-field header errors clear; flip to 1 if range suffers.
// LOCAL 0/1 flag (not an #if vs driver symbols — same include-order trap as the
// header-mode block above; the raw 0x94/0x96 values below expand at point-of-use).
#define OPER_USE_BOOSTED_RX_GAIN  0   // 0 = power-saving 0x94, 1 = boosted 0x96
#if OPER_USE_BOOSTED_RX_GAIN
  #define OPER_RX_GAIN  0x96   // boosted gain (reg 0x08AC)
#else
  #define OPER_RX_GAIN  0x94   // power-saving gain (reg 0x08AC, default)
#endif

// ── Ack-wait timeouts = RETRANSMIT interval when no ACK arrives ───────────────
// This is how long ackReception() waits before RE-SENDING a command — NOT how long
// it can receive an ack (the RX window stays open the whole time, and the starter
// PUSH-delivers a deferred ON/OFF ack via remProtoTick). A retransmit that races a
// legitimately-slow deferred ack is HARMLESS: the starter dedups it (stays silent
// mid-defer, then re-sends the cached ack once the defer resolves).
//
// Per-command, because the round-trip differs ~4× by command type. AUTO-SCALED from
// OPER_TOA_MS so the windows track the active profile (no more hardcoded SF12 numbers):
//   window = 2×OPER_TOA_MS (cmd up + ack down) + starter defer + round-trip margin.
// The starter defers a MOTOR ack to confirm CT current: ON up to 6 s, OFF up to 4 s (relay
// fires immediately). Those defers (REM_ON_ACK_MS / REM_OFF_ACK_MS in the starter's
// remProto.h) are motor/contactor physics, NOT airtime — kept fixed below.
// The +margin covers the 50 ms TX→RX turnaround + starter LCD/processing + slack; a
// retransmit that races a slow-but-legit deferred ack is HARMLESS (starter dedups it).
//   For SF11 explicit (OPER_TOA_MS=720): STATUS≈2.34 s, OFF≈5.84 s, ON≈7.84 s.
//   Worst-case deferred ack lands at 2×0.66 + defer + 0.05 → each window clears it by ~0.47 s.
#define ACK_WAIT_DEFER_OFF_MS  4000UL   // mirrors starter REM_OFF_ACK_MS
#define ACK_WAIT_DEFER_ON_MS   6000UL   // mirrors starter REM_ON_ACK_MS
#define ACK_WAIT_STATUS_MS  ((2UL * OPER_TOA_MS) + 900UL)
#define ACK_WAIT_OFF_MS     ((2UL * OPER_TOA_MS) + ACK_WAIT_DEFER_OFF_MS + 400UL)
#define ACK_WAIT_ON_MS      ((2UL * OPER_TOA_MS) + ACK_WAIT_DEFER_ON_MS  + 400UL)
// Flat ACK_WAIT_MS retained as the MAX of the three, used only to size the radio RX
// listen window (RX_TIMEOUT_MS = ACK_WAIT_MS + 1500 in sx1268Main.h) so the window
// always outlasts the longest per-command retransmit interval.
#define ACK_WAIT_MS   ACK_WAIT_ON_MS
#define OPER_SYNC_MSB     0x34
#define OPER_SYNC_LSB     0x44
#define OPER_TX_POWER     22

// ── Pairing packet types ──
#define PKT_PAIR_REQ      0x0A   // Starter → Gateway
#define PKT_PAIR_ACK      0x0B   // Gateway → Starter
#define PKT_PAIR_DONE     0x0C   // Starter → Gateway
#define PKT_REM_PAIR_REQ     0x0D   // Remote  → Starter  [type][rem_serial:20]
#define PKT_REM_PAIR_ACK     0x0E   // Starter → Remote   [type][starter_id:20][sf][bwCode][cr][pre][pwr]
#define PKT_REM_PAIR_DONE    0x0F   // Remote  → Starter  [type][serial_echo:4]
#define PKT_REM_PAIR_CONFIRM 0x10   // Starter → Remote   [type][serial_echo:4] — final confirm

// ── Pairing timing ──
#define PAIR_BEACON_INTERVAL_MS  2000UL
#define PAIR_SCAN_WINDOW_MS     60000UL
#define PAIR_ACK_TIMEOUT_MS      5000UL
#define PAIR_ACK_MAX_RETRIES         3
// 0x0F→0x10 confirm handshake: retry 0x0F until the Starter's 0x10 confirm arrives.
// This runs on the FAST SF7 pair channel (~150 ms round trip), NOT the SF11 operational
// channel — so the retry interval is short. 4 retries ≈ ~6 s before reverting.
#define PAIR_DONE_RETRY_MS       1500UL
#define PAIR_DONE_MAX_RETRIES        4

// ── Remote Binary Protocol (RBP) ────────────────────────────────────────────
// 0x22/0x23 are collision-free: not 0x0A-0x0F pairing, not 0x01-0x21 GW, not 0x30-0x46 AES-hex
#define PKT_REM_CMD       0x22   // Remote→Starter: [0x22][ E(MAGIC0,MAGIC1,cmd,   0x00 ) ]  5 B (v2)
#define PKT_REM_RSP       0x23   // Starter→Remote: [0x23][ E(MAGIC0,MAGIC1,status,light) ]  5 B (v2)
// CMD codes
#define REM_CMD_M1_ON     0x01
#define REM_CMD_M1_OFF    0x02
#define REM_CMD_STATUS    0x03
#define REM_CMD_RELAY_ON  0x04
#define REM_CMD_RELAY_OFF 0x05
// RSP status codes
#define REM_STA_OFF          0x00
#define REM_STA_ON           0x01
#define REM_STA_BLK_BYPASS   0x02
#define REM_STA_BLK_VFAULT   0x03
#define REM_STA_FAULT        0x04

// ── Plain RBP v2 (fixed-encryption CMD/RSP — replaces seq/MAC; v1 archived) ────
// 5-byte frame: [type][ AES-CTR(per-unit key, fixed IV){ MAGIC0, MAGIC1, value, value2 } ].
// MAGIC (both bytes) MUST match the starter.
#define RBP_MAGIC0    0x5A
#define RBP_MAGIC1    0xC3
                     