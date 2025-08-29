#pragma once

#include "button.h"

#ifndef LOCKSTATE_DEFINED
#define LOCKSTATE_DEFINED
enum class LockState { Unlocked, Locked };
#endif

class LockController {
   public:
    LockController(const char* name, HalButton& keyButton,
                   HalButton& unlockButton, HalValve& lockValve);

    void update();
    void reset();  // 复位函数：解锁并关闭阀门
    LockState getState() const;
    const char* getName() const;

   private:
    const char* name;
    HalButton& keyButton;       // 钥匙按钮传感器
    HalButton& unlockButton;    // 解锁按钮
    HalValve& lockValve;        // 锁控制阀门
    LockState state;
};