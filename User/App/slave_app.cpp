#include "SlaveDevice.h"
#include "cmsis_os2.h"
#include "elog.h"
#include "gpio.h"
#include "main.h"
#include "uwb_task.h"

extern void UWB_Task_Init(void);

const char *TAG = "slave_app";

using namespace SlaveApp;

// Static pointer to the global SlaveDevice instance
static SlaveDevice* g_global_slave_device = nullptr;

// C wrapper function to get synchronized timestamp from SlaveDevice
extern "C" uint32_t slave_device_get_sync_timestamp_ms(void* device) {
    if (device != nullptr) {
        SlaveDevice* slaveDevice = static_cast<SlaveDevice*>(device);
        return slaveDevice->getSyncTimestampMs();
    }
    return 0;
}

extern "C" int main_app(void) {
    UWB_Task_Init();

    // HAL_GPIO_WritePin(UWB_EN_GPIO_Port, UWB_EN_Pin, GPIO_PIN_SET);
    // HAL_GPIO_WritePin(UWB_RST_GPIO_Port, UWB_RST_Pin, GPIO_PIN_SET);

    // SlaveDevice slaveDevice;
    
    // // Register SlaveDevice with easylogger for synchronized timestamps
    // g_global_slave_device = &slaveDevice;
    // elog_set_slave_device(&slaveDevice, slave_device_get_sync_timestamp_ms);
    
    // elog_i(TAG, "SlaveDevice registered with easylogger for synchronized timestamps");
    
    // slaveDevice.run();

    for (;;) {
        elog_i(TAG, "slave_app running");
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        osDelay(500);
    }
    return 0;
}