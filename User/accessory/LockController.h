#pragma once

#include "button.h"

#ifndef LOCKCONTROLLER_HPP
#define LOCKCONTROLLER_HPP
enum class LockState
{
    Unlocked,
    Locked
};
#endif

class LockController
{
  public:
    LockController(const char *name, HalButton &keyButton, HalButton &unlockButton, HalValve &lockValve);

    void Update();
    void Reset(); // 复位函数：解锁并关闭阀门
    [[nodiscard]] LockState GetState() const;
    [[nodiscard]] const char *GetName() const;

  private:
    const char *m_name;
    HalButton &m_keyButton;    // 钥匙按钮传感器
    HalButton &m_unlockButton; // 解锁按钮
    HalValve &m_lockValve;     // 锁控制阀门
    LockState m_state;
};