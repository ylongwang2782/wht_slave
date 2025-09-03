#include "tim.h"

#ifdef __cplusplus
extern "C"
{
#endif

#include "FreeRTOS.h"
#include "hptimer.hpp"
#include "task.h"

#ifdef __cplusplus
}
#endif

static volatile bool s_initialized = false;

uint32_t HptimerGetUs(void)
{
    return __HAL_TIM_GET_COUNTER(&htim2); // 返回当前计数值（1μs 单位）
}

uint32_t HptimerGetMs(void)
{
    return HptimerGetUs() / 1000;
}

uint64_t HptimerGetUs64(void)
{
    TickType_t tick_count = xTaskGetTickCount(); // FreeRTOS 毫秒
    uint32_t us_counter = HptimerGetUs();        // 当前微秒（0~999）
    return (uint64_t)tick_count * 1000ULL + (us_counter % 1000);
}

uint32_t HptimerElapsedUs(const uint32_t refTime)
{
    if (const uint32_t now = HptimerGetUs(); now >= refTime)
    {
        return now - refTime;
    }
    else
    {
        return (0xFFFFFFFF - refTime) + now + 1;
    }
}

bool HptimerIsTimeoutUs(uint32_t refTime, uint32_t timeoutUs)
{
    return HptimerElapsedUs(refTime) >= timeoutUs;
}

void HptimerDelayUs(uint32_t us)
{
    if (!s_initialized || us == 0)
        return;

    uint32_t start = HptimerGetUs();
    while (HptimerElapsedUs(start) < us)
    {
        if (us > 1000 && HptimerElapsedUs(start) % 100 == 0)
        {
            taskYIELD();
        }
    }
}
