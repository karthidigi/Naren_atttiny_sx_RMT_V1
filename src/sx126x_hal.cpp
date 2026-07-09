// =============================================================================
// sx126x_hal.cpp — Arduino / ATtiny1606 HAL for the Semtech SX126x driver
// =============================================================================
// Implements the four user-supplied callbacks declared in sx126x_hal.h
// (write / read / reset / wakeup) on top of the Arduino SPI library.
//
// Board: ATtiny1606 remote (megaTinyCore). Single SX1262 (E22-900M22S) radio on
// fixed pins, so the opaque `context` pointer is unused — the HAL talks to the
// one radio via the pin macros below.
//
// Pin macros are duplicated from hwPins.h on purpose: hwPins.h defines a
// non-inline hwPinInit() under only `#pragma once`, so including it from this
// second translation unit would multiply-define that symbol at link time.
// KEEP THESE IN SYNC WITH hwPins.h (SX1268_NSS / BUSY / RESET_PIN).
// =============================================================================

#include <Arduino.h>
#include <SPI.h>

extern "C" {
#include "sx126x_hal.h"
}

// ── Radio control pins (must match hwPins.h) ────────────────────────────────
#define RADIO_NSS    PIN_PA4   // SX1268_NSS   (LLCC68_NSS)
#define RADIO_BUSY   PIN_PA5   // SX1268_BUSY  (LLCC68_BUSY)
#define RADIO_RESET  PIN_PA6   // SX1268_RESET_PIN (LLCC68_RESET)

// SPI clock: half the CPU clock, as the old SX1268_driver used for AVR
// (SX126x tolerates up to 16 MHz; F_CPU/2 = 10 MHz @ 20 MHz core).
#define RADIO_SPI_FREQ   (F_CPU / 2)

// BUSY-wait ceiling. MUST stay below the ATtiny1606 WDT period (4096 ms) so a
// stuck BUSY line aborts the transfer (returning ERROR) before the watchdog
// resets the MCU — preserves the old driver's SX1268_BUSY_TIMEOUT = 3000 ms.
#define RADIO_BUSY_TIMEOUT_MS  3000UL

// Returns true once BUSY is low, false on timeout.
static bool radio_wait_busy(void)
{
    uint32_t t = millis();
    while (digitalRead(RADIO_BUSY) == HIGH)
    {
        if (millis() - t > RADIO_BUSY_TIMEOUT_MS) return false;
    }
    return true;
}

extern "C" {

sx126x_hal_status_t sx126x_hal_write(const void* context, const uint8_t* command, const uint16_t command_length,
                                     const uint8_t* data, const uint16_t data_length)
{
    (void) context;
    if (!radio_wait_busy()) return SX126X_HAL_STATUS_ERROR;

    digitalWrite(RADIO_NSS, LOW);
    SPI.beginTransaction(SPISettings(RADIO_SPI_FREQ, MSBFIRST, SPI_MODE0));
    for (uint16_t i = 0; i < command_length; i++) SPI.transfer(command[i]);
    for (uint16_t i = 0; i < data_length;    i++) SPI.transfer(data[i]);
    SPI.endTransaction();
    digitalWrite(RADIO_NSS, HIGH);

    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_read(const void* context, const uint8_t* command, const uint16_t command_length,
                                    uint8_t* data, const uint16_t data_length)
{
    (void) context;
    if (!radio_wait_busy()) return SX126X_HAL_STATUS_ERROR;

    digitalWrite(RADIO_NSS, LOW);
    SPI.beginTransaction(SPISettings(RADIO_SPI_FREQ, MSBFIRST, SPI_MODE0));
    // The driver packs the opcode + any status/NOP placeholder bytes into `command`,
    // so we clock those out first, then read `data_length` response bytes with NOPs.
    for (uint16_t i = 0; i < command_length; i++) SPI.transfer(command[i]);
    for (uint16_t i = 0; i < data_length;    i++) data[i] = SPI.transfer(SX126X_NOP);
    SPI.endTransaction();
    digitalWrite(RADIO_NSS, HIGH);

    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_reset(const void* context)
{
    (void) context;
    pinMode(RADIO_RESET, OUTPUT);
    digitalWrite(RADIO_RESET, LOW);
    delayMicroseconds(500);            // datasheet min reset pulse is ~100 µs
    digitalWrite(RADIO_RESET, HIGH);
    delayMicroseconds(100);
    radio_wait_busy();                 // wait for the chip to leave POR/reset
    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_wakeup(const void* context)
{
    (void) context;
    // A falling edge on NSS wakes the chip from sleep; then it raises BUSY until
    // the configuration is restored (warm start). Wait for BUSY to fall.
    digitalWrite(RADIO_NSS, LOW);
    delayMicroseconds(20);
    digitalWrite(RADIO_NSS, HIGH);
    radio_wait_busy();
    return SX126X_HAL_STATUS_OK;
}

}  // extern "C"
