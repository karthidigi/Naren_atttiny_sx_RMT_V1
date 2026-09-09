#define debugSerial Serial

#ifdef SERIAL_DEBUG
  #define DEBUG_PRINT(x) debugSerial.print(x)
  #define DEBUG_PRINTN(x) debugSerial.println(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTN(x)
#endif


void hwSerialInit() {
  #ifdef SERIAL_DEBUG
    debugSerial.begin(SERIAL_BAUD);
    debugSerial.setTimeout(SERIAL_TIMEOUT);
  #endif
}
