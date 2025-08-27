#include "tim.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "hptimer.h"
#include "task.h"


#ifdef __cplusplus
}
#endif

static volatile bool s_initialized = false;

uint32_t hal_hptimer_get_us(void) {
    return __HAL_TIM_GET_COUNTER(&htim2);    // 返回当前计数值（1μs 单位）
}

uint32_t hal_hptimer_get_ms(void) { return hal_hptimer_get_us() / 1000; }

uint64_t hal_hptimer_get_us64(void) {
    TickType_t tick_count = xTaskGetTickCount();    // FreeRTOS 毫秒
    uint32_t us_counter = hal_hptimer_get_us();     // 当前微秒（0~999）
    return (uint64_t)tick_count * 1000ULL + (us_counter % 1000);
}

uint32_t hal_hptimer_elapsed_us(uint32_t ref_time) {
    uint32_t now = hal_hptimer_get_us();
    if (now >= ref_time) {
        return now - ref_time;
    } else {
        return (0xFFFFFFFF - ref_time) + now + 1;
    }
}

bool hal_hptimer_is_timeout_us(uint32_t ref_time, uint32_t timeout_us) {
    return hal_hptimer_elapsed_us(ref_time) >= timeout_us;
}

void hal_hptimer_delay_us(uint32_t us) {
    if (!s_initialized || us == 0) return;

    uint32_t start = hal_hptimer_get_us();
    while (hal_hptimer_elapsed_us(start) < us) {
        if (us > 1000 && hal_hptimer_elapsed_us(start) % 100 == 0) {
            taskYIELD();
        }
    }
}
