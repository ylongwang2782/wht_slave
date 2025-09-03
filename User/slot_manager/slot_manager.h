#pragma once

#include <cstdint>
#include <functional>

/**
 * 时隙类型枚举
 */
enum class SlotType : uint8_t
{
    INACTIVE = 0, // 非激活时隙 - 只读取数据
    ACTIVE = 1    // 激活时隙 - 输出高电平并读取数据
};

/**
 * 时隙信息结构体
 */
struct SlotInfo
{
    uint16_t m_currentSlot;    // 当前时隙编号
    uint16_t m_totalSlots;     // 总时隙数
    SlotType m_slotType;       // 当前时隙类型
    uint8_t m_activePin;       // 如果是激活时隙，对应的引脚编号（逻辑引脚）
    uint32_t m_slotIntervalMs; // 时隙间隔（毫秒）

    SlotInfo() : m_currentSlot(0), m_totalSlots(0), m_slotType(SlotType::INACTIVE), m_activePin(0), m_slotIntervalMs(0)
    {
    }
};

/**
 * 时隙回调函数类型
 * @param slotInfo 当前时隙信息
 */
using SlotCallback = std::function<void(const SlotInfo &slotInfo)>;

/**
 * 同步时间回调函数类型 - 用于获取同步时间
 */
using SyncTimeCallback = std::function<uint64_t()>;

/**
 * 时隙管理器类
 * 负责管理时隙调度，提供时隙切换事件通知
 */
class SlotManager
{
  public:
    SlotManager();
    ~SlotManager() = default;

    // 删除拷贝构造函数和赋值操作符
    SlotManager(const SlotManager &) = delete;
    SlotManager &operator=(const SlotManager &) = delete;

    /**
     * 配置时隙管理器
     * @param startSlot 本设备的起始时隙编号
     * @param deviceSlotCount 本设备负责的时隙数量
     * @param totalSlotCount 整个系统的总时隙数
     * @param slotIntervalMs 时隙间隔（毫秒）
     * @return 配置是否成功
     */
    bool Configure(uint16_t startSlot, uint8_t deviceSlotCount, uint16_t totalSlotCount, uint32_t slotIntervalMs);

    /**
     * 配置时隙管理器（支持单周期模式）
     * @param startSlot 本设备的起始时隙编号
     * @param deviceSlotCount 本设备负责的时隙数量
     * @param totalSlotCount 整个系统的总时隙数
     * @param slotIntervalMs 时隙间隔（毫秒）
     * @param singleCycle 是否为单周期模式（完成一个周期后自动停止）
     * @return 配置是否成功
     */
    bool Configure(uint16_t startSlot, uint8_t deviceSlotCount, uint16_t totalSlotCount, uint32_t slotIntervalMs,
                   bool singleCycle);

    /**
     * 开始时隙调度
     * @return 是否成功启动
     */
    bool Start();

    /**
     * 停止时隙调度
     */
    void Stop();

    /**
     * 处理时隙状态机（需要定期调用）
     */
    void Process();

    /**
     * 设置时隙回调函数
     * @param callback 时隙切换时的回调函数
     */
    void SetSlotCallback(SlotCallback callback);

    /**
     * 设置同步时间回调函数
     * @param callback 获取同步时间的回调函数
     */
    void SetSyncTimeCallback(SyncTimeCallback callback);

    /**
     * 获取当前时隙信息
     * @return 当前时隙信息
     */
    const SlotInfo &GetCurrentSlotInfo() const
    {
        return m_CurrentSlotInfo;
    }

    /**
     * 检查是否正在运行
     * @return 是否正在运行
     */
    bool IsRunning() const
    {
        return m_IsRunning;
    }

    /**
     * 获取总时隙数
     * @return 总时隙数
     */
    uint16_t GetTotalSlots() const
    {
        return m_TotalSlotCount;
    }

    /**
     * 获取当前时隙编号
     * @return 当前时隙编号
     */
    uint16_t GetCurrentSlot() const
    {
        return m_CurrentSlotInfo.m_currentSlot;
    }

    /**
     * 检查当前时隙是否为本设备的激活时隙
     * @return 是否为激活时隙
     */
    bool IsCurrentSlotActive() const
    {
        return m_CurrentSlotInfo.m_slotType == SlotType::ACTIVE;
    }

    /**
     * 获取当前激活的引脚编号（仅在激活时隙有效）
     * @return 激活引脚编号，如果非激活时隙返回0xFF
     */
    uint8_t GetCurrentActivePin() const;

  private:
    // 配置参数
    uint16_t m_StartSlot;      // 本设备的起始时隙编号
    uint8_t m_DeviceSlotCount; // 本设备负责的时隙数量
    uint16_t m_TotalSlotCount; // 整个系统的总时隙数
    uint32_t m_SlotIntervalMs; // 时隙间隔（毫秒）

    // 运行状态
    bool m_IsRunning;           // 是否正在运行
    bool m_IsConfigured;        // 是否已配置
    bool m_IsFirstProcess;      // 是否是第一次process调用
    bool m_SingleCycleMode;     // 是否为单周期模式
    bool m_CycleCompleted;      // 是否已完成一个周期
    uint64_t m_LastSlotTimeUs;  // 上次时隙切换时间（微秒）
    uint64_t m_StartTimeUs;     // 时隙调度开始的绝对时间（微秒）
    SlotInfo m_CurrentSlotInfo; // 当前时隙信息

    // 回调函数
    SlotCallback m_SlotCallback;         // 时隙回调
    SyncTimeCallback m_SyncTimeCallback; // 同步时间回调

    /**
     * 获取当前同步时间（微秒）
     * @return 同步时间
     */
    uint64_t GetCurrentSyncTimeUs();

    /**
     * 计算时隙类型和相关信息
     * @param slotNumber 时隙编号
     * @return 时隙类型
     */
    SlotType CalculateSlotType(uint16_t slotNumber);

    /**
     * 切换到新时隙
     * @param newSlot 新时隙编号
     */
    void SwitchToSlot(uint16_t newSlot);
};