#include "../pid_tick.h"

/* STM32 HAL: HAL_GetTick() is the SysTick millisecond counter.
 * Include the device HAL header for your family before building. */
#include "stm32_hal.h"

uint32_t pid_tick_ms(void) {
    return HAL_GetTick();
}
