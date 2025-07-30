#pragma once

#include "stm32f4xx_hal.h"

// HAL层按钮实现
class HalButton {
   public:
    HalButton(const char* name, GPIO_TypeDef* port, uint16_t pin);

    // 保持原有函数接口
    void update();
    bool isPressed() const;
    bool wasPressed() const;
    const char* getName() const;

   private:
    const char* name;
    GPIO_TypeDef* port;
    uint16_t pin;
    bool lastState;
    bool pressedEdge;
};

// HAL层传感器实现
class HalSensor {
   public:
    HalSensor(const char* name, GPIO_TypeDef* port, uint16_t pin);

    // 保持原有函数接口
    bool isTrigger() const;
    const char* getName() const;

   private:
    const char* name;
    GPIO_TypeDef* port;
    uint16_t pin;
};

// HAL层阀门实现
class HalValve {
   public:
    HalValve(const char* name, GPIO_TypeDef* port, uint16_t pin,
             bool active_high = true);

    // 保持原有函数接口
    void open();
    void close();
    void toggle();
    bool isOpen() const;
    const char* getName() const;

   private:
    const char* name;
    bool active_high;
    GPIO_TypeDef* port;
    uint16_t pin;
    bool isOpenState;
};

// 拨码开关信息结构体
#ifndef DIPSWITCHINFO_DEFINED
#define DIPSWITCHINFO_DEFINED
struct DipSwitchInfo {
    struct PinInfo {
        GPIO_TypeDef* port;
        uint16_t pin;
    };
    PinInfo pins[8];
};
#endif

// HAL层拨码开关实现
class HalDipSwitch {
   public:
    HalDipSwitch(const DipSwitchInfo& info);

    // 保持原有函数接口
    bool isOn(int index) const;
    uint8_t value() const;

   private:
    HalButton keys[8];
};
