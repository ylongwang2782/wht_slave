#include "DeviceStatus.h"

namespace WhtsProtocol {

uint16_t DeviceStatus::toUint16() const {
    uint16_t result = 0;
    if (clrSensor)
        result |= (1 << 0);
    if (sleeveLimit)
        result |= (1 << 1);
    if (electromagnetUnlockButton)
        result |= (1 << 2);
    if (batteryLowAlarm)
        result |= (1 << 3);
    if (pressureSensor)
        result |= (1 << 4);
    if (electromagneticLock1)
        result |= (1 << 5);
    if (electromagneticLock2)
        result |= (1 << 6);
    if (accessory1)
        result |= (1 << 7);
    if (accessory2)
        result |= (1 << 8);
    return result;
}

void DeviceStatus::fromUint16(uint16_t status) {
    clrSensor = (status & (1 << 0)) != 0;
    sleeveLimit = (status & (1 << 1)) != 0;
    electromagnetUnlockButton = (status & (1 << 2)) != 0;
    batteryLowAlarm = (status & (1 << 3)) != 0;
    pressureSensor = (status & (1 << 4)) != 0;
    electromagneticLock1 = (status & (1 << 5)) != 0;
    electromagneticLock2 = (status & (1 << 6)) != 0;
    accessory1 = (status & (1 << 7)) != 0;
    accessory2 = (status & (1 << 8)) != 0;
    reserved = 0;
}

} // namespace WhtsProtocol