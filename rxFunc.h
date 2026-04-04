///////////////////////////////////////
char encapData[MAX_MESSAGE_LEN];
bool liRelRemState = false;   // tracks last known state of starter LiREL relay
///////////////////////////////////////
void rxFunc() {
  msgTxd = 0;
  if (strcmp(encapData, "1N") == 0) {
    // Motor ON confirmed — green LED blinks + low-base tone (~2 s)
    buttonEn[0] = DISABLED;  // M1_ON  (motor running — can't turn on again)
    buttonEn[1] = ENABLED;   // M1_OFF
    buttonEn[2] = ENABLED;   // STA
    watchdogReset();          // tone is ~2 s; WDT is 4 s — reset before blocking
    motorOnTone();            // handles LED blink + buzzer; resets LED on exit

  } else if (strcmp(encapData, "1F") == 0) {
    // Motor OFF confirmed — 2× RED blink + 2× 100 ms buzzer tone
    buttonEn[0] = ENABLED;   // M1_ON
    buttonEn[1] = DISABLED;  // M1_OFF (motor stopped — can't turn off again)
    buttonEn[2] = ENABLED;   // STA
    funcM1LRed(); buzBeep(100); funcLedReset(); delay(100);
    funcM1LRed(); buzBeep(100); funcLedReset();

  } else if (strcmp(encapData, "A0") == 0) {
    // Auto-stop / alarm — same pattern as motor OFF
    buttonEn[0] = ENABLED;   // M1_ON
    buttonEn[1] = DISABLED;  // M1_OFF
    buttonEn[2] = ENABLED;   // STA
    funcM1LRed(); buzBeep(100); funcLedReset(); delay(100);
    funcM1LRed(); buzBeep(100); funcLedReset();

  } else if (strcmp(encapData, "B1") == 0) {
    // Bypass switch active on starter — re-enable all buttons, yellow LED, 3× short beep
    buttonEn[0] = ENABLED;
    buttonEn[1] = ENABLED;
    buttonEn[2] = ENABLED;
    funcM1Yellow();
    buzBeep(100); delay(80);
    buzBeep(100); delay(80);
    buzBeep(100);
    funcLedReset();

  } else if (strcmp(encapData, "F1") == 0) {
    // Voltage fault active on starter — re-enable all buttons, red LED, long beep
    buttonEn[0] = ENABLED;
    buttonEn[1] = ENABLED;
    buttonEn[2] = ENABLED;
    funcM1LRed();
    buzBeep(300); delay(100);
    buzBeep(300);
    funcLedReset();

  } else if (strcmp(encapData, "D0") == 0) {
    // Normal operation restored — brief green flash, no buzzer
    funcM1LGreen(); delay(100); funcLedReset();

  } else if (strcmp(encapData, "D1") == 0) {
    // Fault — RED blink + YELLOW blink + 2× 150 ms equal beeps (alarm pattern)
    buttonEn[0] = ENABLED;
    buttonEn[1] = ENABLED;
    buttonEn[2] = ENABLED;
    funcM1LRed();    buzBeep(150); funcLedReset(); delay(100);
    funcM1Yellow();  buzBeep(150); funcLedReset();

  } else if (strcmp(encapData, "D2") == 0) {
    // Bypass — RED blink + BLUE blink + short/long beeps (asymmetric warning)
    buttonEn[0] = ENABLED;
    buttonEn[1] = ENABLED;
    buttonEn[2] = ENABLED;
    funcM1LRed();    buzBeep(80);  funcLedReset(); delay(100);
    funcStaLBlue();  buzBeep(300); funcLedReset();

  } else if (strcmp(encapData, "L1") == 0) {
    // Light switch ON confirmed by starter
    liRelRemState = true;
    buttonEn[0] = ENABLED;
    buttonEn[1] = ENABLED;
    buttonEn[2] = ENABLED;
    funcM1LGreen();
    buzBeep(100);
    funcLedReset();

  } else if (strcmp(encapData, "L0") == 0) {
    // Light switch OFF confirmed by starter
    liRelRemState = false;
    buttonEn[0] = ENABLED;
    buttonEn[1] = ENABLED;
    buttonEn[2] = ENABLED;
    funcM1LRed();
    buzBeep(100);
    funcLedReset();

  } else {
    // Unknown or noise packet — ignore silently (no LED)
  }
}
