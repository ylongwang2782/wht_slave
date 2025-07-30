#include "button.h"

// HalButton实现
HalButton::HalButton(const char* name, GPIO_TypeDef* port, uint16_t pin)
    : name(name), port(port), pin(pin), lastState(false), pressedEdge(false) {}

void HalButton::update() {
    bool current = isPressed();
    pressedEdge = (!lastState && current);
    lastState = current;
}

bool HalButton::isPressed() const {
    return HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET;    // 假设低电平为按下
}

bool HalButton::wasPressed() const { return pressedEdge; }

const char* HalButton::getName() const { return name; }

// HalSensor实现
HalSensor::HalSensor(const char* name, GPIO_TypeDef* port, uint16_t pin)
    : name(name), port(port), pin(pin) {}

bool HalSensor::isTrigger() const {
    return HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET;
}

const char* HalSensor::getName() const { return name; }

// HalValve实现
HalValve::HalValve(const char* name, GPIO_TypeDef* port, uint16_t pin,
                   bool active_high)
    : name(name),
      active_high(active_high),
      port(port),
      pin(pin),
      isOpenState(false) {}

void HalValve::open() {
    HAL_GPIO_WritePin(port, pin, active_high ? GPIO_PIN_SET : GPIO_PIN_RESET);
    isOpenState = true;
}

void HalValve::close() {
    HAL_GPIO_WritePin(port, pin, active_high ? GPIO_PIN_RESET : GPIO_PIN_SET);
    isOpenState = false;
}

void HalValve::toggle() {
    HAL_GPIO_TogglePin(port, pin);
    isOpenState = !isOpenState;
}

bool HalValve::isOpen() const { return isOpenState; }

const char* HalValve::getName() const { return name; }

// HalDipSwitch实现
HalDipSwitch::HalDipSwitch(const DipSwitchInfo& info)
    : keys{HalButton("DIP0", info.pins[0].port, info.pins[0].pin),
           HalButton("DIP1", info.pins[1].port, info.pins[1].pin),
           HalButton("DIP2", info.pins[2].port, info.pins[2].pin),
           HalButton("DIP3", info.pins[3].port, info.pins[3].pin),
           HalButton("DIP4", info.pins[4].port, info.pins[4].pin),
           HalButton("DIP5", info.pins[5].port, info.pins[5].pin),
           HalButton("DIP6", info.pins[6].port, info.pins[6].pin),
           HalButton("DIP7", info.pins[7].port, info.pins[7].pin)} {}

bool HalDipSwitch::isOn(int index) const {
    if (index < 0 || index >= 8) return false;
    return keys[index].isPressed();
}

uint8_t HalDipSwitch::value() const {
    uint8_t val = 0;
    for (int i = 0; i < 8; ++i) {
        if (isOn(i)) {
            val |= (1 << i);
        }
    }
    return val;
}
