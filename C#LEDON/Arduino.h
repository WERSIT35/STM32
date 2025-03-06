#ifndef WProgram_h
#define WProgram_h

#include <stdint.h>  // For standard types like uint8_t, uint16_t
#include <stdlib.h>  // For standard functions like malloc/free

// Define Arduino-like functions if needed
#define HIGH 1
#define LOW  0

// Stub for millis() - replace with HAL_GetTick()
static inline uint32_t millis() {
    return millis();
}

// Stub for delay() - replace with HAL_Delay()
static inline void delay(uint32_t ms) {
    delay(ms);
}

#endif // WProgram_h
