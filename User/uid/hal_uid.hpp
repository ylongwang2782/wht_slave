#pragma once
class DeviceUID
{
  public:
    static uint32_t get()
    {
        static uint32_t value = [] {
            const auto *uid_address = reinterpret_cast<uint32_t *>(0x1FFF7A10);
            return *uid_address;
        }();
        return value;
    }
    ~DeviceUID() = delete;
    DeviceUID() = delete;
};
