#include "ContinuityCollector.h"
#include "SlaveDevice.h"
#include "cmsis_os2.h"
#include "elog.h"
#include "gpio.h"
#include "uwb_task.h"

extern void UWB_Task_Init(void);

const char *TAG = "slave_app";

using namespace SlaveApp;

extern "C" int main_app(void) {
    UWB_Task_Init();

    SlaveDevice slaveDevice;
    slaveDevice.run();

    for (;;) {
        elog_i(TAG, "slave_app running");
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        osDelay(500);
    }
    return 0;
}