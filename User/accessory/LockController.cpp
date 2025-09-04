#include "LockController.h"

// 单个阀门构造函数（向后兼容）
LockController::LockController(const char *name, HalButton &keyButton, HalButton &unlockButton, HalValve &lockValve)
    : m_name(name), m_keyButton(keyButton), m_unlockButton(unlockButton), m_state(LockState::Unlocked)
{
    m_lockValves.push_back(&lockValve);
}

// 多个阀门构造函数
LockController::LockController(const char *name, HalButton &keyButton, HalButton &unlockButton,
                               std::vector<HalValve *> lockValves)
    : m_name(name), m_keyButton(keyButton), m_unlockButton(unlockButton), m_lockValves(lockValves),
      m_state(LockState::Unlocked)
{
}

void LockController::Update()
{
    // 更新按钮边沿状态
    m_keyButton.Update();
    m_unlockButton.Update();

    // 钥匙按钮按下 → 上锁（打开所有阀门）
    if (m_keyButton.WasPressed())
    {
        for (auto *valve : m_lockValves)
        {
            valve->Open();
        }
        m_state = LockState::Locked;
    }

    // 解锁按钮按下 → 解锁（关闭所有阀门）
    if (m_unlockButton.WasPressed())
    {
        for (auto *valve : m_lockValves)
        {
            valve->Close();
        }
        m_state = LockState::Unlocked;
    }
}

void LockController::Reset()
{
    // 复位操作：解锁并关闭所有阀门
    for (auto *valve : m_lockValves)
    {
        valve->Close();
    }
    m_state = LockState::Unlocked;
}

LockState LockController::GetState() const
{
    return m_state;
}

const char *LockController::GetName() const
{
    return m_name;
}