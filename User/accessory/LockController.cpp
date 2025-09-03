#include "LockController.h"

LockController::LockController(const char *name, HalButton &keyButton, HalButton &unlockButton, HalValve &lockValve)
    : m_name(name), m_keyButton(keyButton), m_unlockButton(unlockButton), m_lockValve(lockValve),
      m_state(LockState::Unlocked)
{
}

void LockController::Update()
{
    // 更新按钮边沿状态
    m_keyButton.Update();
    m_unlockButton.Update();

    // 钥匙按钮按下 → 上锁（打开阀门）
    if (m_keyButton.WasPressed())
    {
        m_lockValve.Open();
        m_state = LockState::Locked;
    }

    // 解锁按钮按下 → 解锁（关闭阀门）
    if (m_unlockButton.WasPressed())
    {
        m_lockValve.Close();
        m_state = LockState::Unlocked;
    }
}

void LockController::Reset()
{
    // 复位操作：解锁并关闭阀门
    m_lockValve.Close();
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