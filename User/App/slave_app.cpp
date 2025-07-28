#include "cmsis_os2.h"
#include "elog.h"
#include "gpio.h"
#include "uwb_task.h"

extern void UWB_Task_Init(void);

const char *TAG = "slave_app";

extern "C" int main_app(void) {
    // elog_i(TAG, "slave_app");
    UWB_Task_Init();

    uint8_t data[] = {0x00,0x01,0x02,0x03};
    for (;;) {
        // Send UWB data with optional delay
        int result = UWB_SendData(data, sizeof(data), 0);
        // elog_i(TAG, "slave_app");
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        osDelay(100);
    }
    return 0;
}