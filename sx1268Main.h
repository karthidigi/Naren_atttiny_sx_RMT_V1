// =============================================
// sx1268Main.h – SX1262 LoRa Radio Driver (Remote ATtiny1606)
// Replaces: llcc68Main.h
// =============================================
// Radio driver: official Semtech SX126x C driver (src/sx126x.c) + Arduino HAL
// (src/sx126x_hal.cpp). Replaces the old hand-rolled src/SX1268_driver.*.
//
// SX1262 hardware notes:
//   - TCXO on DIO3: configured before calibration
//   - DIO2 as RF switch: REQUIRED on the E22-900M22S module (drives its external TX/RX
//     antenna switch) — toggled in sx1268Init() (see TEST B note), identical to the starter
//   - SX126x family: SX1262 and SX1268 share the same SPI command interface
//
// IQ polarity (§15.4): the Semtech driver's sx126x_set_lora_pkt_params() SETS bit 2 of
// reg 0x0736 for standard IQ. On this E22-900M22S that produced sporadic near-field RX
// header errors, so we OVERRIDE it back to the field-proven V2_2 logic (standard IQ →
// bit 2 CLEARED) via radioApplyIqWorkaroundOverride(), called after every
// sx126x_set_lora_pkt_params(). See that helper for the full rationale.
//
// Pairing channel : SF7 /BW125/CR4-5/pre=8 /sync=0x1234/14dBm  (LDRO=0)
// Operational channel: SF11/BW125/CR4-5/pre=12/sync=0x3444/22dBm (LDRO=1)
// Frequency: 910 MHz
// NOTE: Operational RF params MUST MATCH STARTER (AVRDB_LLCC_V2_2_GSM_But_LCD)

#ifndef SX1268_MAIN_H
#define SX1268_MAIN_H

#include "zSettings.h"  // PAIR_*/OPER_*/PKT_* constants
#include "hwPins.h"     // SX1268_NSS, SX1268_BUSY, SX1268_RESET_PIN, SX1268_DIO1
#include <SPI.h>
#include "src/sx126x.h"       // Semtech SX126x driver API
#include "src/sx126x_regs.h"  // SX126X_REG_LR_SYNCWORD / RXGAIN / OCP / IQ_POLARITY

// ── Radio context ────────────────────────────────────────────────────────────
// The Semtech driver passes this opaque pointer to every sx126x_hal_* callback.
// This board has one radio on fixed pins (hwPins.h), so the HAL ignores it; we
// pass the address of this dummy purely for API conformance.
static const uint8_t radioCtx = 0;
#define RADIO (&radioCtx)

volatile bool dio1_triggered = false;

// ── Dynamic operational sync word ────────────────────────────────────────────
// Defaults to the compiled-in constants until a pairing completes and the
// Starter-derived sync word is received in 0x0E and stored in EEPROM.
// Updated by pairRemoteNode.h on successful pairing.
uint8_t g_operSyncMsb = OPER_SYNC_MSB;
uint8_t g_operSyncLsb = OPER_SYNC_LSB;

// ────────────────────────────────────────────────
// IQ-polarity workaround override (§15.4 deviation)
// ────────────────────────────────────────────────
// The Semtech driver applies the datasheet §15.4 inverted-IQ workaround inside
// sx126x_set_lora_pkt_params(): for STANDARD IQ it SETS bit 2 of reg 0x0736.
// On this E22-900M22S module that "audit-correct" behavior reintroduced sporadic
// near-field RX header errors, whereas the original V2_2 firmware (standard IQ →
// bit 2 CLEARED) never produced a single header error. Empirical field behavior
// overrides the datasheet here. Call this AFTER every sx126x_set_lora_pkt_params()
// to put bit 2 back to the field-proven state.
//   bit 2 = 0 for standard IQ → no header errors;  bit 2 = 1 → header errors.
// All channels here use standard IQ, so this always clears bit 2.
static inline void radioApplyIqWorkaroundOverride() {
  uint8_t value = 0;
  sx126x_read_register(RADIO, SX126X_REG_IQ_POLARITY, &value, 1);
  value &= ~(1 << 2);  // standard IQ → clear bit 2 (overrides Semtech's §15.4 set)
  sx126x_write_register(RADIO, SX126X_REG_IQ_POLARITY, &value, 1);
}

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
// remHandleRsp is defined in remProto.h (included after sx1268Main.h)
void remHandleRsp(const uint8_t* buf, uint8_t len);

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
// Actual on-air length of the queued TX frame (RBP CMD=8 / pairing 1-21). Set by
// send_lora_data(); consumed in STATE_TX_SETUP so we transmit only the real bytes
// instead of a fixed 32-byte frame (~halves SF11 airtime). MUST match starter.
static uint8_t tx_length = 32;

// Set when an RX_DONE + CRC_ERR IRQ fires — signals ackReception() to retry fast
volatile bool rxCrcError = false;
volatile bool rxHdrError = false;   // RX header-decode failure → ackReception() fast-retry

static unsigned long state_start_time = 0;
// Timeouts auto-scale with the LoRa profile via OPER_TOA_MS / ACK_WAIT_MS (zSettings.h)
// so SF12's ~2.6 s/packet airtime is accommodated without hardcoded SF11 values.
//   TX: one-packet airtime + 3 s slack.   SF12 → 6.5 s, SF11 → 4.8 s
//   RX: listen 1.5 s past the ack-wait deadline.  SF12 → 13.5 s, SF11 → 10.1 s
static const unsigned long TX_TIMEOUT_MS = OPER_TOA_MS + 400UL;   // real airtime + small stuck-TX margin
static const unsigned long RX_TIMEOUT_MS = ACK_WAIT_MS + 1500UL;
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
  tx_length = length;            // transmit only the real bytes
  radio_state = STATE_TX_SETUP;
  state_start_time = millis();
}

// ────────────────────────────────────────────────
// Channel-switch helpers
// ────────────────────────────────────────────────

// Switch to SF7/BW125/CR4-5/pre=8/sync=0x1234/14dBm  (LDRO=0)
void switchToPairChannel() {
  sx126x_set_standby(RADIO, SX126X_STANDBY_CFG_RC);

  sx126x_mod_params_lora_t mp;
  mp.sf   = (sx126x_lora_sf_t) PAIR_SF;
  mp.bw   = (sx126x_lora_bw_t) PAIR_BW;
  mp.cr   = (sx126x_lora_cr_t) PAIR_CR;
  mp.ldro = 0;                                       // SF7/BW125 → LDRO off
  sx126x_set_lora_mod_params(RADIO, &mp);            // auto-applies §15.1 TX-modulation workaround

  sx126x_pkt_params_lora_t pp;
  pp.preamble_len_in_symb = PAIR_PREAMBLE;
  pp.header_type          = SX126X_LORA_PKT_EXPLICIT;
  pp.pld_len_in_bytes     = 32;
  pp.crc_is_on            = true;
  pp.invert_iq_is_on      = false;                   // standard IQ
  sx126x_set_lora_pkt_params(RADIO, &pp);
  radioApplyIqWorkaroundOverride();                  // E22 field-proven: clear IQ bit 2 (overrides §15.4)

  uint8_t sw[2] = { PAIR_SYNC_MSB, PAIR_SYNC_LSB };
  sx126x_write_register(RADIO, SX126X_REG_LR_SYNCWORD, sw, 2);
  sx126x_set_tx_params(RADIO, PAIR_TX_POWER, SX126X_RAMP_200_US);
  sx126x_clear_irq_status(RADIO, SX126X_IRQ_ALL);
  dio1_triggered = false;
  radio_state = STATE_IDLE;
  state_start_time = millis();
  pairing_mode = true;  // pairing packets (0x0A-0x0F) now expected
  DEBUG_PRINTN("Radio: switched to PAIR channel (SF7/BW125/0x1234)");
}

// Switch to OPER_SF/OPER_BW/CR4-5/pre=12 + dynamic sync word/22dBm
// OPER_LDRO is compile-time: 1 for SF11+BW125, 0 for SF10+BW250.
// Sync word is g_operSyncMsb/g_operSyncLsb (loaded from EEPROM after pairing,
// or defaults to OPER_SYNC_MSB/LSB on factory-fresh devices).
void switchToOperationalChannel() {
  sx126x_set_standby(RADIO, SX126X_STANDBY_CFG_RC);

  sx126x_mod_params_lora_t mp;
  mp.sf   = (sx126x_lora_sf_t) OPER_SF;
  mp.bw   = (sx126x_lora_bw_t) OPER_BW;
  mp.cr   = (sx126x_lora_cr_t) OPER_CR;
  mp.ldro = OPER_LDRO;
  sx126x_set_lora_mod_params(RADIO, &mp);            // auto-applies §15.1 TX-modulation workaround

  sx126x_pkt_params_lora_t pp;
  pp.preamble_len_in_symb = OPER_PREAMBLE;
  pp.header_type          = (sx126x_lora_pkt_len_modes_t) OPER_HEADER_MODE;
  pp.pld_len_in_bytes     = OPER_PAYLOAD_LEN;
  pp.crc_is_on            = true;
  pp.invert_iq_is_on      = false;                   // standard IQ
  sx126x_set_lora_pkt_params(RADIO, &pp);
  radioApplyIqWorkaroundOverride();                  // E22 field-proven: clear IQ bit 2 (overrides §15.4)

  uint8_t sw[2] = { g_operSyncMsb, g_operSyncLsb };
  sx126x_write_register(RADIO, SX126X_REG_LR_SYNCWORD, sw, 2);
  sx126x_set_tx_params(RADIO, OPER_TX_POWER, SX126X_RAMP_200_US);
  sx126x_clear_irq_status(RADIO, SX126X_IRQ_ALL);
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
  // Control pins + SPI bus (the HAL drives NSS/BUSY/RESET; set them up here once).
  pinMode(SX1268_NSS, OUTPUT);
  digitalWrite(SX1268_NSS, HIGH);
  pinMode(SX1268_BUSY, INPUT);
  pinMode(SX1268_RESET_PIN, OUTPUT);
  digitalWrite(SX1268_RESET_PIN, HIGH);
  SPI.begin();

  // Hardware reset (HAL toggles RESET and waits for BUSY to fall)
  sx126x_reset(RADIO);
  _delay_ms(10);  // _delay_ms: F_CPU NOP loop — correct even if millis() is frozen from prior tone()

  sx126x_set_standby(RADIO, SX126X_STANDBY_CFG_RC);

  // TCXO: configure before calibration (640 RTC steps ≈ 10 ms @ 1.8 V).
  // Calibrate AFTER enabling the TCXO so the PLL/IMG blocks lock to it.
  sx126x_set_dio3_as_tcxo_ctrl(RADIO, SX126X_TCXO_CTRL_1_8V, 640);
  sx126x_cal(RADIO, SX126X_CAL_ALL);
  sx126x_set_standby(RADIO, SX126X_STANDBY_CFG_RC);

  sx126x_set_reg_mode(RADIO, SX126X_REG_MODE_DCDC);
  sx126x_cal_img(RADIO, 0xD7, 0xDB);  // 863–870 MHz covers 867 MHz (MUST match carrier below + starter)

  // ⚠️ TEST B (TEMPORARY): DIO2 RF-switch control DISABLED on the remote to match the
  //    known-good V2_2 remote (which never enabled it and never had a header error).
  //    If header errors clear → the V3 remote module has no DIO2-wired switch and enabling
  //    it was mis-toggling a pin. If comms get WORSE (antenna stranded) → the module DOES
  //    need it; REVERT by uncommenting the call below.
  // sx126x_set_dio2_as_rf_sw_ctrl(RADIO, true);   // TEST B: was enabled
  sx126x_set_pkt_type(RADIO, SX126X_PKT_TYPE_LORA);

  // Frequency: 867.1 MHz (MUST equal the starter's LORA_FREQUENCY_HZ + image-cal band above).
  // The Semtech driver converts Hz → PLL steps internally.
  sx126x_set_rf_freq(RADIO, 867100000UL);

  sx126x_pa_cfg_params_t pa;
  pa.pa_duty_cycle = 0x04;
  pa.hp_max        = 0x07;
  pa.device_sel    = 0x00;   // 0x00 = SX1262
  pa.pa_lut        = 0x01;
  sx126x_set_pa_cfg(RADIO, &pa);
  // OCP: MUST be 0x38 (140 mA) for the SX1262/SX1268 HP path (datasheet §13.4.4).
  // Reset default 0x18 (60 mA) current-starves the PA at 22 dBm, silently reducing
  // actual TX power by several dBm and killing range.
  sx126x_set_ocp_value(RADIO, 0x38);
  sx126x_set_tx_params(RADIO, PAIR_TX_POWER, SX126X_RAMP_200_US);

  // Init on PAIR channel (SF7/BW125/CR4-5/LDRO=0/pre=8)
  // pairRemNodeTick() calls switchToPairChannel() on first loop — this just
  // ensures the radio starts in a valid, known state.
  {
    sx126x_mod_params_lora_t mp;
    mp.sf   = (sx126x_lora_sf_t) PAIR_SF;
    mp.bw   = (sx126x_lora_bw_t) PAIR_BW;
    mp.cr   = (sx126x_lora_cr_t) PAIR_CR;
    mp.ldro = 0;
    sx126x_set_lora_mod_params(RADIO, &mp);

    sx126x_pkt_params_lora_t pp;
    pp.preamble_len_in_symb = PAIR_PREAMBLE;
    pp.header_type          = SX126X_LORA_PKT_EXPLICIT;
    pp.pld_len_in_bytes     = 32;
    pp.crc_is_on            = true;
    pp.invert_iq_is_on      = false;
    sx126x_set_lora_pkt_params(RADIO, &pp);
    radioApplyIqWorkaroundOverride();   // clear IQ bit 2 before retention list captures it
  }

  uint8_t sw[2] = { PAIR_SYNC_MSB, PAIR_SYNC_LSB };
  sx126x_write_register(RADIO, SX126X_REG_LR_SYNCWORD, sw, 2);

  sx126x_set_buffer_base_address(RADIO, 0x00, 0x80);
  sx126x_cfg_tx_clamp(RADIO);  // §15.2 better resistance to antenna mismatch (was fixResistanceAntenna)

  // ── RX Gain (reg 0x08AC) — selected by OPER_RX_GAIN in zSettings.h ────────
  // 0x96 = boosted (+3 dB sensitivity) — best range, but can overload the front end at
  // near-field and trigger header errors. 0x94 = power-saving/default (currently selected).
  // MUST match the starter's setting.
  uint8_t rxGain = OPER_RX_GAIN;
  sx126x_write_register(RADIO, SX126X_REG_RXGAIN, &rxGain, 1);

  // ── Retention list: keep RXGAIN + TX_MODULATION + IQ_POLARITY across sleep ─
  // sx126x_init_retention_list() registers exactly these 3 registers (0x08AC,
  // 0x0889, 0x0736) so RX boosted gain / TX-modulation / IQ-polarity survive
  // STBY→SLEEP instead of reverting to reset defaults after every sleep.
  sx126x_init_retention_list(RADIO);

  sx126x_irq_mask_t irq_mask = SX126X_IRQ_TX_DONE | SX126X_IRQ_RX_DONE | SX126X_IRQ_TIMEOUT | OPER_HDR_ERR_IRQ | SX126X_IRQ_CRC_ERROR;
  sx126x_set_dio_irq_params(RADIO, irq_mask, irq_mask, SX126X_IRQ_NONE, SX126X_IRQ_NONE);

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
        sx126x_set_standby(RADIO, SX126X_STANDBY_CFG_RC);
        // Re-arm packet params. Pair channel always EXPLICIT (variable-length pairing PDUs).
        // Oper channel uses OPER_HEADER_MODE (implicit trial) with fixed OPER_PAYLOAD_LEN.
        // To revert oper to explicit: change OPER_HEADER_MODE / OPER_PAYLOAD_LEN in zSettings.h.
        {
          uint16_t                    _pre = pairing_mode ? PAIR_PREAMBLE          : OPER_PREAMBLE;
          sx126x_lora_pkt_len_modes_t _hdr = pairing_mode ? SX126X_LORA_PKT_EXPLICIT
                                                          : (sx126x_lora_pkt_len_modes_t) OPER_HEADER_MODE;
          uint8_t                     _len = pairing_mode ? tx_length              : OPER_PAYLOAD_LEN;
          sx126x_pkt_params_lora_t pp;
          pp.preamble_len_in_symb = _pre;
          pp.header_type          = _hdr;
          pp.pld_len_in_bytes     = _len;
          pp.crc_is_on            = true;
          pp.invert_iq_is_on      = false;
          sx126x_set_lora_pkt_params(RADIO, &pp);
          radioApplyIqWorkaroundOverride();  // E22 field-proven: clear IQ bit 2 (overrides §15.4)
        }
        sx126x_set_buffer_base_address(RADIO, 0x00, 0x80);
        sx126x_write_buffer(RADIO, 0x00, bufferRadio, tx_length);
        dio1_triggered = false;
        sx126x_clear_irq_status(RADIO, SX126X_IRQ_ALL);
        sx126x_set_tx(RADIO, 0);  // 0 = no timeout (single TX)
        radio_state = STATE_TX_WAIT;
        state_start_time = millis();
        break;
      }

    case STATE_TX_WAIT:
      {
        if (dio1_triggered) {
          last_radio_activity = millis();  // TX Done IRQ also proves radio is alive
          sx126x_clear_irq_status(RADIO, SX126X_IRQ_ALL);
          dio1_triggered = false;
          radio_state = STATE_RX_SETUP;
          state_start_time = millis();
        } else if (millis() - state_start_time >= TX_TIMEOUT_MS) {
          sx126x_clear_irq_status(RADIO, SX126X_IRQ_ALL);
          radio_state = STATE_IDLE;
          state_start_time = millis();
        }
        break;
      }

    case STATE_RX_SETUP:
      {
        // Timed RX window (restored from V2_3). After timed RX timeout/done the radio
        // returns to STDBY_RC automatically, so we re-issue SetRx via STATE_RX_SETUP each
        // cycle. The periodic TIMEOUT IRQ also proves the radio + ISR are alive, which
        // feeds the watchdog below (continuous RX produced no such liveness signal and
        // caused spurious idle re-inits). Explicit standby first ensures a known state.
        sx126x_set_standby(RADIO, SX126X_STANDBY_CFG_RC);
        dio1_triggered = false;
        // No per-RX packet-param setup: the active params (from switchToOperationalChannel /
        // prior TX_SETUP) already carry the right header mode and length. In IMPLICIT mode the
        // RX length is fixed at OPER_PAYLOAD_LEN (5 B), correct because both CMD and RSP are 5 B;
        // in EXPLICIT mode the on-air header self-describes the length. Nothing to re-arm here.
        sx126x_set_buffer_base_address(RADIO, 0x00, 0x80);
        sx126x_clear_irq_status(RADIO, SX126X_IRQ_ALL);
        // Defensive: re-apply the configured RX gain (OPER_RX_GAIN) on every RX entry
        // in case the SX1262's internal AGC reset reverted 0x08AC.
        {
          uint8_t _rxGain = OPER_RX_GAIN;
          sx126x_write_register(RADIO, SX126X_REG_RXGAIN, &_rxGain, 1);
        }
        sx126x_set_rx(RADIO, RX_TIMEOUT_MS);   // timed window in ms (driver converts to RTC steps)
        radio_state = STATE_RX_WAIT;
        state_start_time = millis();
        break;
      }

    case STATE_RX_WAIT:
      {
        if (dio1_triggered) {
          last_radio_activity = millis();  // any IRQ = radio + ISR alive
          sx126x_irq_mask_t irq_status = 0;
          sx126x_get_irq_status(RADIO, &irq_status);

          // FIX: Only process packet when RX_DONE AND no CRC error.
          // SX1268 sets both RX_DONE and CRC_ERR together on bad packets.
          if ((irq_status & SX126X_IRQ_RX_DONE) && !(irq_status & SX126X_IRQ_CRC_ERROR)) {
            sx126x_rx_buffer_status_t rxbs;
            sx126x_get_rx_buffer_status(RADIO, &rxbs);
            uint8_t rx_length = rxbs.pld_len_in_bytes;
            uint8_t rx_start  = rxbs.buffer_start_pointer;
            sx126x_stop_rtc(RADIO);  // §15.4: stop RTC + clear timeout event after timed RX (was fixRxTimeout)

            if (rx_length > 0 && rx_length <= 32) {
              uint8_t rx_buffer[32];
              sx126x_read_buffer(RADIO, rx_start, rx_buffer, rx_length);

              // Pairing packets only on the pair channel (pairing_mode). Range includes
              // 0x10 (PKT_REM_PAIR_CONFIRM) so the final pairing confirm is routed.
              if (pairing_mode && rx_buffer[0] >= PKT_PAIR_REQ && rx_buffer[0] <= PKT_REM_PAIR_CONFIRM) {
                dispatchPairPkt(rx_buffer, rx_length);
              } else if (rx_buffer[0] == PKT_REM_RSP) {
                remHandleRsp(rx_buffer, rx_length);
              } else {
                DEBUG_PRINTN("RX: ignored");
              }
            }
          } else if (irq_status & SX126X_IRQ_CRC_ERROR) {
            // Preamble+header OK but payload CRC failed — packet reached us corrupted.
            DEBUG_PRINTN("RX CRC Err");
            rxCrcError = true;  // signal ackReception() to fast-retry
          } else if (irq_status & SX126X_IRQ_HEADER_ERROR) {
            // Preamble detected but header could not be decoded (weak signal/interference).
            // A reply likely reached us corrupted → fast-retry like a CRC error instead of
            // waiting the full ACK window.
            DEBUG_PRINTN("RX Hdr Err");
            rxHdrError = true;
          } else if (irq_status & SX126X_IRQ_TIMEOUT) {
            // Timed RX window expired with no packet — no ACK received.
            DEBUG_PRINTN("RX Timeout");
          }

          // Timed RX (not continuous): after any event the radio is in STDBY_RC.
          // Must go through STATE_RX_SETUP to re-issue SetRx.
          sx126x_clear_irq_status(RADIO, SX126X_IRQ_ALL);
          dio1_triggered = false;
          radio_state = STATE_IDLE;
          state_start_time = millis();
        } else if (millis() - state_start_time >= RX_TIMEOUT_MS) {
          sx126x_clear_irq_status(RADIO, SX126X_IRQ_ALL);
          radio_state = STATE_IDLE;
          state_start_time = millis();
        }
        break;
      }
  }

  // ── Remote radio watchdog ─────────────────────────────────────────────
  // Timed RX (window above) fires a TIMEOUT IRQ via DIO1 every cycle, refreshing
  // last_radio_activity. If 60 s pass with no DIO1 event at all, the ISR or radio
  // hardware has locked up — full re-init and return to the operational channel.
  if (millis() - last_radio_activity > REMOTE_WATCHDOG_MS) {
    sx1268Init();
    switchToOperationalChannel();
    last_radio_activity = millis();
  }
}

#endif  // SX1268_MAIN_H
