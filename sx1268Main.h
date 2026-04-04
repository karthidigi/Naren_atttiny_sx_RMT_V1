// =============================================
// sx1268Main.h – SX1262 LoRa Radio Driver (Remote ATtiny1606)
// Replaces: llcc68Main.h
// =============================================
// SX1262 hardware notes:
//   - TCXO on DIO3: configured before calibration
//   - DIO2 as RF switch: NOT present on SX1262 — do NOT call setDio2AsRfSwitchCtrl()
//   - SX126x family: SX1262 and SX1268 share the same SPI command interface
//
// Pairing channel : SF7 /BW125/CR4-5/pre=8 /sync=0x1234/14dBm  (LDRO=0)
// Operational channel: SF11/BW125/CR4-5/pre=12/sync=0x3444/22dBm (LDRO=1)
// Frequency: 900 MHz
// NOTE: Operational RF params MUST MATCH STARTER (AVRDB_LLCC_V2_2_GSM_But_LCD)

#ifndef SX1268_MAIN_H
#define SX1268_MAIN_H

#include "zSettings.h"  // PAIR_*/OPER_*/PKT_* constants
#include "hwPins.h"     // SX1268_NSS, SX1268_BUSY, SX1268_RESET_PIN, SX1268_DIO1
#include <SPI.h>
#include "src/SX1268_driver.h"

volatile bool dio1_triggered = false;

// ── Dynamic operational sync word ────────────────────────────────────────────
// Defaults to the compiled-in constants until a pairing completes and the
// Starter-derived sync word is received in 0x0E and stored in EEPROM.
// Updated by pairRemoteNode.h on successful pairing.
uint8_t g_operSyncMsb = OPER_SYNC_MSB;
uint8_t g_operSyncLsb = OPER_SYNC_LSB;

// ────────────────────────────────────────────────
// Forward declarations
// ────────────────────────────────────────────────
void send_lora_data(const uint8_t* data, uint8_t length);
void sx1268Init();
void sx1268Func();
void switchToPairChannel();
void switchToOperationalChannel();
// dispatchPairPkt is defined in pairRemoteNode.h (included after sx1268Main.h)
void dispatchPairPkt(const uint8_t* buf, uint8_t len);

// ────────────────────────────────────────────────
// Radio state machine
// ────────────────────────────────────────────────
enum RadioState {
  STATE_IDLE,
  STATE_TX_SETUP,
  STATE_TX_WAIT,
  STATE_RX_SETUP,
  STATE_RX_WAIT
};

// ────────────────────────────────────────────────
// Globals
// ────────────────────────────────────────────────
static uint8_t radioBuf[32];  // shared TX/RX buffer
#define payload radioBuf      // alias for TX
#define bufferRadio radioBuf  // alias for RX

// Set when an RX_DONE + CRC_ERR IRQ fires — signals ackReception() to retry fast
volatile bool rxCrcError = false;

static unsigned long state_start_time = 0;
static const unsigned long TX_TIMEOUT_MS = 5000UL;    // SF11/BW125 ToA ~1.1 s + 3.9 s margin
static const unsigned long RX_TIMEOUT_MS = 8000UL;    // ACK delay 1.5s + ACK ToA ~1.1s + 5.4s margin
static const unsigned long IDLE_DURATION_MS = 100UL;  // Fast return to RX mode

static RadioState radio_state = STATE_IDLE;
// true while on pair channel (0x1234); false in operational mode (0x3444).
// Gates pairing-packet byte-range check in STATE_RX_WAIT.
static bool pairing_mode = false;

// ────────────────────────────────────────────────
// ISR – native PORT interrupt (ATtiny1606 PA7)
// PORTA_PORT_vect fires for any pin in PORT A.
// Check INTFLAGS to confirm PA7, then clear by writing 1 to the flag bit.
// ────────────────────────────────────────────────
ISR(PORTA_PORT_vect) {
  if (PORTA.INTFLAGS & PIN7_bm) {
    PORTA.INTFLAGS = PIN7_bm;  // clear interrupt flag (write 1 to clear)
    dio1_triggered = true;
  }
}

// ────────────────────────────────────────────────
// Non-blocking send (same interface as before)
// ────────────────────────────────────────────────
void send_lora_data(const uint8_t* data, uint8_t length) {
  if (length > 32) return;
  if (!bit_is_set(SREG, 7)) sei();
  memset(payload, 0, sizeof(payload));
  memcpy(payload, data, length);
  radio_state = STATE_TX_SETUP;
  state_start_time = millis();
}

// ────────────────────────────────────────────────
// Channel-switch helpers
// ────────────────────────────────────────────────

// Switch to SF7/BW125/CR4-5/pre=8/sync=0x1234/14dBm  (LDRO=0)
void switchToPairChannel() {
  SX1268_setStandby(SX1268_STANDBY_RC);
  SX1268_setModulationParamsLoRa(PAIR_SF, PAIR_BW, PAIR_CR, 0);  // LDRO=0
  SX1268_setPacketParamsLoRa(PAIR_PREAMBLE, SX1268_HEADER_EXPLICIT, 32,
                             SX1268_CRC_ON, SX1268_IQ_STANDARD);
  // SX126x errata: re-apply IQ register after every setPacketParams call
  SX1268_fixInvertedIq(SX1268_IQ_STANDARD);
  uint8_t sw[2] = { PAIR_SYNC_MSB, PAIR_SYNC_LSB };
  SX1268_writeRegister(SX1268_REG_LORA_SYNC_WORD_MSB, sw, 2);
  SX1268_setTxParams(PAIR_TX_POWER, SX1268_PA_RAMP_200U);
  SX1268_clearIrqStatus(SX1268_IRQ_ALL);
  dio1_triggered = false;
  radio_state = STATE_IDLE;
  state_start_time = millis();
  pairing_mode = true;  // pairing packets (0x0A-0x0F) now expected
  DEBUG_PRINTN("Radio: switched to PAIR channel (SF7/BW125/0x1234)");
}

// Switch to SF11/BW125/CR4-5/pre=12 + dynamic sync word/22dBm  (LDRO=1)
// Sync word is g_operSyncMsb/g_operSyncLsb (loaded from EEPROM after pairing,
// or defaults to OPER_SYNC_MSB/LSB on factory-fresh devices).
void switchToOperationalChannel() {
  SX1268_setStandby(SX1268_STANDBY_RC);
  SX1268_setModulationParamsLoRa(OPER_SF, OPER_BW, OPER_CR, 1);  // LDRO=1
  SX1268_setPacketParamsLoRa(OPER_PREAMBLE, SX1268_HEADER_EXPLICIT, 32,
                             SX1268_CRC_ON, SX1268_IQ_STANDARD);
  // SX126x errata: re-apply IQ register after every setPacketParams call
  SX1268_fixInvertedIq(SX1268_IQ_STANDARD);
  uint8_t sw[2] = { g_operSyncMsb, g_operSyncLsb };
  SX1268_writeRegister(SX1268_REG_LORA_SYNC_WORD_MSB, sw, 2);
  SX1268_setTxParams(OPER_TX_POWER, SX1268_PA_RAMP_200U);
  SX1268_clearIrqStatus(SX1268_IRQ_ALL);
  dio1_triggered = false;
  radio_state = STATE_IDLE;
  state_start_time = millis();
  pairing_mode = false;  // operational: AES-encrypted packets only
  DEBUG_PRINTN("Radio: switched to OPER channel (dyn sync)");
}

// ────────────────────────────────────────────────
// Radio init
// ────────────────────────────────────────────────
void sx1268Init() {
  // Register SPI bus and pins with driver
  SX1268_setSPI(SPI, 0);
  SX1268_setPins(SX1268_NSS, SX1268_BUSY);

  SX1268_begin();

  // Hardware reset
  SX1268_reset(SX1268_RESET_PIN);
  delay(10);

  SX1268_setStandby(SX1268_STANDBY_RC);

  // TCXO: configure before calibration
  SX1268_setDio3AsTcxoCtrl(SX1268_DIO3_OUTPUT_1_8, SX1268_TCXO_DELAY_10);
  SX1268_calibrate(0xFF);
  SX1268_setStandby(SX1268_STANDBY_RC);

  SX1268_setRegulatorMode(SX1268_REGULATOR_DC_DC);
  // SX1268_calibrateImage(SX1268_CAL_IMG_902, SX1268_CAL_IMG_928);

  // After  — 863–870 MHz (EU868, covers 867 MHz exactly)
  SX1268_calibrateImage(SX1268_CAL_IMG_863, SX1268_CAL_IMG_870);

  // NOTE: SX1262 has NO internal DIO2 RF switch — do NOT call setDio2AsRfSwitchCtrl()
  SX1268_setPacketType(SX1268_LORA_MODEM);

  // Frequency: 900 MHz
  // uint32_t rfFreq = ((uint64_t)900000000UL << SX1268_RF_FREQUENCY_SHIFT) / SX1268_RF_FREQUENCY_XTAL;

  // After
  uint32_t rfFreq = ((uint64_t)867000000UL << SX1268_RF_FREQUENCY_SHIFT) / SX1268_RF_FREQUENCY_XTAL;
  SX1268_setRfFrequency(rfFreq);

  SX1268_setPaConfig(0x04, 0x07, 0x00, 0x01);
  // OCP: MUST be set to 0x38 (140 mA) for SX1262/SX1268 HP path (datasheet §13.4.4).
  // Reset default is 0x18 (60 mA) which current-starves the PA at 22 dBm, silently
  // reducing actual TX power by several dBm and killing range.
  uint8_t ocp = 0x38;
  SX1268_writeRegister(SX1268_REG_OCP_CONFIGURATION, &ocp, 1);
  SX1268_setTxParams(PAIR_TX_POWER, SX1268_PA_RAMP_200U);

  // Init on PAIR channel (SF7/BW125/CR4-5/LDRO=0/pre=8)
  // pairRemNodeTick() calls switchToPairChannel() on first loop — this just
  // ensures the radio starts in a valid, known state.
  SX1268_setModulationParamsLoRa(PAIR_SF, PAIR_BW, PAIR_CR, 0);  // LDRO=0

  SX1268_setPacketParamsLoRa(PAIR_PREAMBLE, SX1268_HEADER_EXPLICIT, 32,
                             SX1268_CRC_ON, SX1268_IQ_STANDARD);
  SX1268_fixInvertedIq(SX1268_IQ_STANDARD);

  uint8_t sw[2] = { PAIR_SYNC_MSB, PAIR_SYNC_LSB };
  SX1268_writeRegister(SX1268_REG_LORA_SYNC_WORD_MSB, sw, 2);

  SX1268_setBufferBaseAddress(0x00, 0x80);
  SX1268_fixResistanceAntenna();

  uint16_t irq_mask = SX1268_IRQ_TX_DONE | SX1268_IRQ_RX_DONE | SX1268_IRQ_TIMEOUT | SX1268_IRQ_HEADER_ERR | SX1268_IRQ_CRC_ERR;
  SX1268_setDioIrqParams(irq_mask, irq_mask,
                         SX1268_IRQ_NONE, SX1268_IRQ_NONE);

  // DIO1 rising-edge interrupt — native PORT register (ATtiny1606 PA7)
  PORTA.DIRCLR = PIN7_bm;               // PA7 as input
  PORTA.PIN7CTRL = PORT_ISC_RISING_gc;  // rising-edge sense; enables pin interrupt

  radio_state = STATE_IDLE;
  state_start_time = millis();
  dio1_triggered = false;

  // Load dynamic sync word from EEPROM (written during pairing).
  // Falls back to compiled-in defaults if not yet paired.
  loadOperSyncWord(&g_operSyncMsb, &g_operSyncLsb);
}

// ────────────────────────────────────────────────
// State machine (called every loop iteration)
// ────────────────────────────────────────────────
void sx1268Func() {
  // Watchdog: full re-init if no DIO1 IRQ fires for 60 s.
  // With timed RX (8 s window) the chip fires a TIMEOUT IRQ every cycle,
  // so DIO1 triggers at least every ~8 s in normal operation.
  // 60 s of silence means the ISR or radio hardware has locked up.
  static unsigned long last_radio_activity = 0;
  static const unsigned long REMOTE_WATCHDOG_MS = 60000UL;

  switch (radio_state) {

    case STATE_IDLE:
      if (millis() - state_start_time >= IDLE_DURATION_MS) {
        radio_state = STATE_RX_SETUP;
        state_start_time = millis();
      }
      break;

    case STATE_TX_SETUP:
      {
        // FIX: Always go through standby before TX.
        // If radio is in timed RX, issuing SetTx directly can silently drop
        // the opcode (BUSY briefly HIGH during RX→TX transition).
        SX1268_setStandby(SX1268_STANDBY_RC);
        SX1268_setBufferBaseAddress(0x00, 0x80);
        SX1268_writeBuffer(0x00, bufferRadio, 32);
        dio1_triggered = false;
        SX1268_clearIrqStatus(SX1268_IRQ_ALL);
        SX1268_setTx(SX1268_TX_SINGLE);
        radio_state = STATE_TX_WAIT;
        state_start_time = millis();
        break;
      }

    case STATE_TX_WAIT:
      {
        if (dio1_triggered) {
          last_radio_activity = millis();  // TX Done IRQ also proves radio is alive
          SX1268_clearIrqStatus(SX1268_IRQ_ALL);
          dio1_triggered = false;
          radio_state = STATE_RX_SETUP;
          state_start_time = millis();
        } else if (millis() - state_start_time >= TX_TIMEOUT_MS) {
          SX1268_clearIrqStatus(SX1268_IRQ_ALL);
          radio_state = STATE_IDLE;
          state_start_time = millis();
        }
        break;
      }

    case STATE_RX_SETUP:
      {
        dio1_triggered = false;
        SX1268_setBufferBaseAddress(0x00, 0x80);
        SX1268_clearIrqStatus(SX1268_IRQ_ALL);
        // 8-second RX window
        SX1268_setRx((uint32_t)(RX_TIMEOUT_MS * 64UL));
        radio_state = STATE_RX_WAIT;
        state_start_time = millis();
        break;
      }

    case STATE_RX_WAIT:
      {
        if (dio1_triggered) {
          last_radio_activity = millis();  // any IRQ = radio + ISR alive
          uint16_t irq_status;
          SX1268_getIrqStatus(&irq_status);

          // FIX: Only process packet when RX_DONE AND no CRC error.
          // SX1268 sets both RX_DONE and CRC_ERR together on bad packets.
          if ((irq_status & SX1268_IRQ_RX_DONE) && !(irq_status & SX1268_IRQ_CRC_ERR)) {
            uint8_t rx_length, rx_start;
            SX1268_getRxBufferStatus(&rx_length, &rx_start);
            SX1268_fixRxTimeout();

            if (rx_length > 0 && rx_length <= 32) {
              uint8_t rx_buffer[32];
              SX1268_readBuffer(rx_start, rx_buffer, rx_length);

              // Only treat as pairing packet when on pair channel.
              // In operational mode (sync=0x3444) the radio cannot receive
              // pairing packets (sync=0x1234). Without this guard an AES-
              // encrypted ACK whose first byte is 0x0A-0x0F would be
              // silently misrouted and rxFunc() would never run.
              if (pairing_mode && rx_buffer[0] >= PKT_PAIR_REQ && rx_buffer[0] <= PKT_REM_PAIR_DONE) {
                dispatchPairPkt(rx_buffer, rx_length);
              } else {
                decryptNFunc(rx_buffer, rx_length);
              }
            }
          } else if (irq_status & SX1268_IRQ_CRC_ERR) {
            // Received preamble+header OK but payload CRC failed.
            // Means packet reached us but was corrupted (range boundary / interference).
            DEBUG_PRINTN("RX CRC Err");
            rxCrcError = true;  // signal ackReception() to fast-retry
          } else if (irq_status & SX1268_IRQ_HEADER_ERR) {
            // Preamble detected but header could not be decoded.
            // Usually means very weak signal or wrong SF/BW on sender.
            DEBUG_PRINTN("RX Hdr Err");
          } else if (irq_status & SX1268_IRQ_TIMEOUT) {
            // Timed RX window expired with no packet — no ACK received.
            DEBUG_PRINTN("RX Timeout");
          }

          // Timed RX (not continuous): after any event the radio is in STDBY_RC.
          // Must go through STATE_RX_SETUP to re-issue SetRx.
          SX1268_clearIrqStatus(SX1268_IRQ_ALL);
          dio1_triggered = false;
          radio_state = STATE_IDLE;
          state_start_time = millis();
        } else if (millis() - state_start_time >= RX_TIMEOUT_MS) {
          SX1268_clearIrqStatus(SX1268_IRQ_ALL);
          radio_state = STATE_IDLE;
          state_start_time = millis();
        }
        break;
      }
  }

  // ── Remote radio watchdog ─────────────────────────────────────────────
  // Timed RX (8 s window) fires a TIMEOUT IRQ via DIO1 every cycle.
  // If 60 s pass with no DIO1 event at all, the ISR or radio has locked up.
  // Perform a full re-init and return to the operational channel.
  if (millis() - last_radio_activity > REMOTE_WATCHDOG_MS) {
    sx1268Init();
    switchToOperationalChannel();
    last_radio_activity = millis();
  }
}

#endif  // SX1268_MAIN_H
