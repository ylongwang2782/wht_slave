#include "cmsis_os2.h"
#include "elog.h"
#include "gpio.h"
#include "uwb_task.h"

extern void UWB_Task_Init(void);

const char *TAG = "slave_app";

extern "C" int main_app(void) {
    // elog_i(TAG, "slave_app");
    UWB_Task_Init();

    for (;;) {
        elog_i(TAG, "slave_app");
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        osDelay(10);
    }
    return 0;
}