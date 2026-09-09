#define debugSerial Serial

// Variadic, so a second Print argument (HEX/DEC) works the same way it does on the
// starter. The single-argument form silently rejected DEBUG_PRINTN(x, HEX).
#ifdef SERIAL_DEBUG
  #define DEBUG_PRINT(...)  debugSerial.print(__VA_ARGS__)
  #define DEBUG_PRINTN(...) debugSerial.println(__VA_ARGS__)
#else
  #define DEBUG_PRINT(...)  do { } while (0)
  #define DEBUG_PRINTN(...) do { } while (0)
#endif


void hwSerialInit() {
  #ifdef SERIAL_DEBUG
    debugSerial.begin(SERIAL_BAUD);
    debugSerial.setTimeout(SERIAL_TIMEOUT);
  #endif
}
