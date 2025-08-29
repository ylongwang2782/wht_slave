#include "LockController.h"

LockController::LockController(const char* name, HalButton& keyButton,
                               HalButton& unlockButton, HalValve& lockValve)
    : name(name),
      keyButton(keyButton),
      unlockButton(unlockButton),
      lockValve(lockValve),
      state(LockState::Unlocked) {}

void LockController::update() {
    // 更新按钮边沿状态
    keyButton.update();
    unlockButton.update();

    // 钥匙按钮按下 → 上锁（打开阀门）
    if (keyButton.wasPressed()) {
        lockValve.open();
        state = LockState::Locked;
    }

    // 解锁按钮按下 → 解锁（关闭阀门）
    if (unlockButton.wasPressed()) {
        lockValve.close();
        state = LockState::Unlocked;
    }
}

void LockController::reset() {
    // 复位操作：解锁并关闭阀门
    lockValve.close();
    state = LockState::Unlocked;
}

LockState LockController::getState() const { return state; }

const char* LockController::getName() const { return name; }