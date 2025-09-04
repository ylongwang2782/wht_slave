#pragma once

#include "button.h"
#include <vector>

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
    // 单个阀门构造函数（向后兼容）
    LockController(const char *name, HalButton &keyButton, HalButton &unlockButton, HalValve &lockValve);

    // 多个阀门构造函数
    LockController(const char *name, HalButton &keyButton, HalButton &unlockButton, std::vector<HalValve *> lockValves);

    void Update();
    void Reset(); // 复位函数：解锁并关闭所有阀门
    [[nodiscard]] LockState GetState() const;
    [[nodiscard]] const char *GetName() const;

  private:
    const char *m_name;
    HalButton &m_keyButton;               // 钥匙按钮传感器
    HalButton &m_unlockButton;            // 解锁按钮
    std::vector<HalValve *> m_lockValves; // 锁控制阀门列表
    LockState m_state;
};