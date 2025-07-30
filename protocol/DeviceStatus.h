#ifndef WHTS_PROTOCOL_DEVICE_STATUS_H
#define WHTS_PROTOCOL_DEVICE_STATUS_H

#include <cstdint>

namespace WhtsProtocol {

// 设备状态位结构
struct DeviceStatus {
    bool clrSensor : 1;
    bool sleeveLimit : 1;
    bool electromagnetUnlockButton : 1;
    bool batteryLowAlarm : 1;
    bool pressureSensor : 1;
    bool electromagneticLock1 : 1;
    bool electromagneticLock2 : 1;
    bool accessory1 : 1;
    bool accessory2 : 1;
    uint8_t reserved : 7;

    uint16_t toUint16() const;
    void fromUint16(uint16_t status);
};

} // namespace WhtsProtocol

#endif // WHTS_PROTOCOL_DEVICE_STATUS_H