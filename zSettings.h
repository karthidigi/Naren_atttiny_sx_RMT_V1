#pragma once
/////////////////////////////////////////////////////
#define FIRMWARE_VERSION        "ATtiny1606"
#define HARDWARE_VERSION        "1.1.1"

/////////////////////////////////////////////////////
// #define SERIAL_DEBUG   // uncomment for debug output via Serial (UART2)
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
#define PAIR_BW           SX1268_BW_125000
#define PAIR_CR           SX1268_CR_4_5
#define PAIR_PREAMBLE     8
#define PAIR_SYNC_MSB     0x12
#define PAIR_SYNC_LSB     0x34
#define PAIR_TX_POWER     14
// LDRO: SF7+BW125 → symbol time 1.0 ms < 16 ms → OFF (literal 0)

// ── Operational LoRa profile (compile-time; pick ONE) ──────────────────────
// Must match the paired Starter profile on the same installation.
// SF11+BW125 : long range, ToA ~1053 ms / 8-byte RBP pkt, LDRO ON
// SF10+BW250 : ~4× faster (ToA ~242 ms), LDRO OFF, ~3 dB less link budget
#define LORA_PROFILE_SF10_BW250
// #define LORA_PROFILE_SF11_BW125

#if defined(LORA_PROFILE_SF11_BW125)
  #define OPER_SF        11
  #define OPER_BW        SX1268_BW_125000
  #define OPER_BW_CODE   1
  #define OPER_LDRO      1    // SF11+BW125 symbol time 16.4 ms → LDRO ON
#elif defined(LORA_PROFILE_SF10_BW250)
  #define OPER_SF        10
  #define OPER_BW        SX1268_BW_250000
  #define OPER_BW_CODE   2
  #define OPER_LDRO      0    // SF10+BW250 symbol time 4.1 ms → LDRO OFF
#else
  #error "zSettings.h: define LORA_PROFILE_SF11_BW125 or LORA_PROFILE_SF10_BW250"
#endif

#define OPER_CR           SX1268_CR_4_5
#define OPER_PREAMBLE     12
#define OPER_SYNC_MSB     0x34
#define OPER_SYNC_LSB     0x44
#define OPER_TX_POWER     22

// ── Pairing packet types ──
#define PKT_PAIR_REQ      0x0A   // Starter → Gateway
#define PKT_PAIR_ACK      0x0B   // Gateway → Starter
#define PKT_PAIR_DONE     0x0C   // Starter → Gateway
#define PKT_REM_PAIR_REQ  0x0D   // Remote  → Starter  [type][rem_serial:20]
#define PKT_REM_PAIR_ACK  0x0E   // Starter → Remote   [type][starter_id:20][sf][bwCode][cr][pre][pwr]
#define PKT_REM_PAIR_DONE 0x0F   // Remote  → Starter  [type][serial_echo:4]

// ── Pairing timing ──
#define PAIR_BEACON_INTERVAL_MS  2000UL
#define PAIR_SCAN_WINDOW_MS     60000UL
#define PAIR_ACK_TIMEOUT_MS      5000UL
#define PAIR_ACK_MAX_RETRIES         3

// ── Remote Binary Protocol (RBP) ────────────────────────────────────────────
// 0x22/0x23 are collision-free: not 0x0A-0x0F pairing, not 0x01-0x21 GW, not 0x30-0x46 AES-hex
#define PKT_REM_CMD       0x22   // Remote→Starter: [0x22][cmd][idHi][idLo][seq][mac0][mac1][mac2]
#define PKT_REM_RSP       0x23   // Starter→Remote: [0x23][status][flags][seqEcho][mac0][mac1][mac2]
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
                     