#include <Arduino.h>
#include "../pid_tick.h"

/* Arduino / ESP32 Arduino core. */
extern "C" uint32_t pid_tick_ms(void) {
    return (uint32_t)millis();
}
