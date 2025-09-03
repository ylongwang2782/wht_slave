#include "slot_manager.h"
#include "elog.h"
#include "hptimer.hpp"

SlotManager::SlotManager()
    : m_StartSlot(0), m_DeviceSlotCount(0), m_TotalSlotCount(0), m_SlotIntervalMs(0), m_IsRunning(false),
      m_IsConfigured(false), m_IsFirstProcess(true), m_SingleCycleMode(false), m_CycleCompleted(false),
      m_LastSlotTimeUs(0), m_StartTimeUs(0)
{
    elog_v("SlotManager", "SlotManager constructed");
}

bool SlotManager::Configure(uint16_t startSlot, uint8_t deviceSlotCount, uint16_t totalSlotCount,
                            uint32_t slotIntervalMs)
{
    if (m_IsRunning)
    {
        elog_e("SlotManager", "Cannot configure while running");
        return false;
    }

    if (deviceSlotCount == 0 || totalSlotCount == 0 || slotIntervalMs == 0)
    {
        elog_e("SlotManager", "Invalid configuration parameters");
        return false;
    }

    if (startSlot >= totalSlotCount)
    {
        elog_e("SlotManager", "Start slot %d exceeds total slots %d", startSlot, totalSlotCount);
        return false;
    }

    if (startSlot + deviceSlotCount > totalSlotCount)
    {
        elog_e("SlotManager", "Device slots exceed total slots");
        return false;
    }

    m_StartSlot = startSlot;
    m_DeviceSlotCount = deviceSlotCount;
    m_TotalSlotCount = totalSlotCount;
    m_SlotIntervalMs = slotIntervalMs;
    m_SingleCycleMode = false; // 默认为连续模式
    m_CycleCompleted = false;
    m_IsConfigured = true;

    // 初始化时隙信息
    m_CurrentSlotInfo.m_totalSlots = m_TotalSlotCount;
    m_CurrentSlotInfo.m_slotIntervalMs = m_SlotIntervalMs;

    elog_v("SlotManager", "Configured - Start: %d, Count: %d, Total: %d, Interval: %dms", startSlot, deviceSlotCount,
           totalSlotCount, slotIntervalMs);

    return true;
}

bool SlotManager::Configure(uint16_t startSlot, uint8_t deviceSlotCount, uint16_t totalSlotCount,
                            uint32_t slotIntervalMs, bool singleCycle)
{
    if (m_IsRunning)
    {
        elog_e("SlotManager", "Cannot configure while running");
        return false;
    }

    if (deviceSlotCount == 0 || totalSlotCount == 0 || slotIntervalMs == 0)
    {
        elog_e("SlotManager", "Invalid configuration parameters");
        return false;
    }

    if (startSlot >= totalSlotCount)
    {
        elog_e("SlotManager", "Start slot %d exceeds total slots %d", startSlot, totalSlotCount);
        return false;
    }

    if (startSlot + deviceSlotCount > totalSlotCount)
    {
        elog_e("SlotManager", "Device slots exceed total slots");
        return false;
    }

    m_StartSlot = startSlot;
    m_DeviceSlotCount = deviceSlotCount;
    m_TotalSlotCount = totalSlotCount;
    m_SlotIntervalMs = slotIntervalMs;
    m_SingleCycleMode = singleCycle;
    m_CycleCompleted = false;
    m_IsConfigured = true;

    // 初始化时隙信息
    m_CurrentSlotInfo.m_totalSlots = m_TotalSlotCount;
    m_CurrentSlotInfo.m_slotIntervalMs = m_SlotIntervalMs;

    elog_v("SlotManager", "Configured - Start: %d, Count: %d, Total: %d, Interval: %dms, SingleCycle: %s", startSlot,
           deviceSlotCount, totalSlotCount, slotIntervalMs, singleCycle ? "Yes" : "No");

    return true;
}

bool SlotManager::Start()
{
    if (!m_IsConfigured)
    {
        elog_e("SlotManager", "Cannot start - not configured");
        return false;
    }

    if (m_IsRunning)
    {
        elog_w("SlotManager", "Already running");
        return true;
    }

    m_IsRunning = true;
    m_IsFirstProcess = true;  // 标记为第一次process调用
    m_CycleCompleted = false; // 重置周期完成标志

    // 记录时隙调度开始的绝对时间
    m_StartTimeUs = GetCurrentSyncTimeUs();
    m_LastSlotTimeUs = m_StartTimeUs;

    // 设置初始时隙为0，但不立即触发回调
    m_CurrentSlotInfo.m_currentSlot = 0;
    m_CurrentSlotInfo.m_slotType = CalculateSlotType(0);
    m_CurrentSlotInfo.m_totalSlots = m_TotalSlotCount;
    m_CurrentSlotInfo.m_slotIntervalMs = m_SlotIntervalMs;

    if (m_CurrentSlotInfo.m_slotType == SlotType::ACTIVE)
    {
        // 计算激活的引脚编号（逻辑引脚）
        m_CurrentSlotInfo.m_activePin = 0 - m_StartSlot;
        elog_v("SlotManager", "Initial ACTIVE slot 0, pin %d", m_CurrentSlotInfo.m_activePin);
    }
    else
    {
        m_CurrentSlotInfo.m_activePin = 0xFF; // 无效引脚
        elog_v("SlotManager", "Initial INACTIVE slot 0");
    }

    elog_v("SlotManager", "Started slot management from slot 0");
    return true;
}

void SlotManager::Stop()
{
    if (!m_IsRunning)
    {
        return;
    }

    m_IsRunning = false;
    elog_v("SlotManager", "Stopped slot management");
}

void SlotManager::Process()
{
    if (!m_IsRunning)
    {
        return;
    }

    // 如果是第一次process调用，立即处理第0个时隙
    if (m_IsFirstProcess)
    {
        m_IsFirstProcess = false;
        // 立即触发第0个时隙的处理
        if (m_SlotCallback)
        {
            m_SlotCallback(m_CurrentSlotInfo);
        }
        elog_v("SlotManager", "First process call, handled slot 0");
        return;
    }

    uint64_t currentTimeUs = GetCurrentSyncTimeUs();
    uint64_t slotIntervalUs = m_SlotIntervalMs * 1000ULL;

    // 基于绝对时间计算当前应该处于哪个时隙
    uint64_t elapsedFromStartUs = currentTimeUs - m_StartTimeUs;
    uint64_t totalCycles = elapsedFromStartUs / slotIntervalUs;
    uint16_t expectedSlot = totalCycles % m_TotalSlotCount;

    // 检查单周期模式下是否已完成一个完整周期
    if (m_SingleCycleMode && !m_CycleCompleted && totalCycles >= m_TotalSlotCount)
    {
        m_CycleCompleted = true;
        elog_v("SlotManager", "Single cycle completed, stopping slot management");
        Stop();
        return;
    }

    // 检查是否需要切换到新的时隙
    if (expectedSlot != m_CurrentSlotInfo.m_currentSlot)
    {
        // 可能跳过了多个时隙，直接切换到正确的时隙
        SwitchToSlot(expectedSlot);

        // 计算这个时隙的理论开始时间（避免累积误差）
        uint64_t slotCycles = elapsedFromStartUs / slotIntervalUs;
        m_LastSlotTimeUs = m_StartTimeUs + slotCycles * slotIntervalUs;

        elog_v("SlotManager", "Absolute time sync - Expected slot: %d, Elapsed: %lu us, Cycle: %lu", expectedSlot,
               (unsigned long)elapsedFromStartUs, (unsigned long)totalCycles);
    }
}

void SlotManager::SetSlotCallback(SlotCallback callback)
{
    m_SlotCallback = callback;
}

void SlotManager::SetSyncTimeCallback(SyncTimeCallback callback)
{
    m_SyncTimeCallback = callback;
}

uint8_t SlotManager::GetCurrentActivePin() const
{
    if (m_CurrentSlotInfo.m_slotType == SlotType::ACTIVE)
    {
        return m_CurrentSlotInfo.m_activePin;
    }
    return 0xFF; // 无效引脚号
}

uint64_t SlotManager::GetCurrentSyncTimeUs()
{
    if (m_SyncTimeCallback)
    {
        return m_SyncTimeCallback();
    }
    // 如果没有同步时间回调，使用本地时间
    return HptimerGetUs();
}

SlotType SlotManager::CalculateSlotType(uint16_t slotNumber)
{
    // 检查是否在本设备的时隙范围内
    if (slotNumber >= m_StartSlot && slotNumber < m_StartSlot + m_DeviceSlotCount)
    {
        return SlotType::ACTIVE;
    }
    return SlotType::INACTIVE;
}

void SlotManager::SwitchToSlot(uint16_t newSlot)
{
    m_CurrentSlotInfo.m_currentSlot = newSlot;
    m_CurrentSlotInfo.m_slotType = CalculateSlotType(newSlot);

    if (m_CurrentSlotInfo.m_slotType == SlotType::ACTIVE)
    {
        m_CurrentSlotInfo.m_activePin = newSlot - m_StartSlot;
        elog_v("SlotManager", "Switched to ACTIVE slot %d, pin %d", newSlot, m_CurrentSlotInfo.m_activePin);
    }
    else
    {
        m_CurrentSlotInfo.m_activePin = 0xFF; // 无效引脚
        elog_v("SlotManager", "Switched to INACTIVE slot %d", newSlot);
    }

    // 触发回调
    if (m_SlotCallback)
    {
        m_SlotCallback(m_CurrentSlotInfo);
    }
}