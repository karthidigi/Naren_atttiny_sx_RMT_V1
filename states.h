#pragma once
#define ENABLED 1
#define DISABLED 0
////////////////////////////////////////
// 3 buttons: M1_ON(0), M1_OFF(1), STA(2)
//
// Gating model (always-enabled motor buttons):
//   buttonEn[0]/[1] no longer track the starter's motor status — both are set
//   ENABLED once the device is paired/operational (boot / pairing / wake) and
//   stay that way. ON is additionally gated by msgTxd (no pre-empt); OFF
//   pre-empts an in-flight command (stop always wins) — see button.h.
uint8_t buttonEn[3] = { DISABLED, DISABLED, ENABLED };
bool msgTxd = 0;
// REM_CMD_* of the last command sent; 0 = none. Lives here (not button.h) so
// remProto.h can consult it for the OFF-pre-empt stale-ACK guard.
uint8_t lastTxCmdCode = 0;
