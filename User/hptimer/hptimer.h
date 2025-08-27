#pragma once

#include <stdbool.h>
#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 获取当前时间（单位：微秒）
 * @return 以微秒为单位的 32 位计数值（1μs 精度）
 */
uint32_t hal_hptimer_get_us(void);

/**
 * @brief 获取当前时间（单位：毫秒）
 * @return 以毫秒为单位的时间
 */
uint32_t hal_hptimer_get_ms(void);

/**
 * @brief 获取当前时间（单位：微秒，64位）
 * @return 高精度时间戳（μs），结合 FreeRTOS tick 和定时器
 */
uint64_t hal_hptimer_get_us64(void);

/**
 * @brief 获取从某一参考时间起已经经过的时间（μs）
 * @param ref_time 起始时间（通过 hal_hptimer_get_us() 获取）
 * @return 已经过的时间（μs）
 */
uint32_t hal_hptimer_elapsed_us(uint32_t ref_time);

/**
 * @brief 判断某个超时时间是否已到
 * @param ref_time 起始时间
 * @param timeout_us 超时时间（μs）
 * @return true 表示已超时
 */
bool hal_hptimer_is_timeout_us(uint32_t ref_time, uint32_t timeout_us);

/**
 * @brief 延时指定的微秒数（会 busy-wait）
 * @param us 延时时间（μs）
 */
void hal_hptimer_delay_us(uint32_t us);

#ifdef __cplusplus
}
#endif
