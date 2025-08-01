#pragma once

#include <cstdint>
#include <functional>

/**
 * 时隙类型枚举
 */
enum class SlotType : uint8_t {
    INACTIVE = 0,    // 非激活时隙 - 只读取数据
    ACTIVE = 1       // 激活时隙 - 输出高电平并读取数据
};

/**
 * 时隙信息结构体
 */
struct SlotInfo {
    uint8_t currentSlot;        // 当前时隙编号
    uint8_t totalSlots;         // 总时隙数
    SlotType slotType;          // 当前时隙类型
    uint8_t activePin;          // 如果是激活时隙，对应的引脚编号（逻辑引脚）
    uint32_t slotIntervalMs;    // 时隙间隔（毫秒）
    
    SlotInfo() : currentSlot(0), totalSlots(0), slotType(SlotType::INACTIVE), 
                 activePin(0), slotIntervalMs(0) {}
};

/**
 * 时隙回调函数类型
 * @param slotInfo 当前时隙信息
 */
using SlotCallback = std::function<void(const SlotInfo& slotInfo)>;

/**
 * 同步时间回调函数类型 - 用于获取同步时间
 */
using SyncTimeCallback = std::function<uint64_t()>;

/**
 * 时隙管理器类
 * 负责管理时隙调度，提供时隙切换事件通知
 */
class SlotManager {
public:
    SlotManager();
    ~SlotManager() = default;

    // 删除拷贝构造函数和赋值操作符
    SlotManager(const SlotManager&) = delete;
    SlotManager& operator=(const SlotManager&) = delete;

    /**
     * 配置时隙管理器
     * @param startSlot 本设备的起始时隙编号
     * @param deviceSlotCount 本设备负责的时隙数量
     * @param totalSlotCount 整个系统的总时隙数
     * @param slotIntervalMs 时隙间隔（毫秒）
     * @return 配置是否成功
     */
    bool configure(uint8_t startSlot, uint8_t deviceSlotCount, 
                   uint8_t totalSlotCount, uint32_t slotIntervalMs);

    /**
     * 开始时隙调度
     * @return 是否成功启动
     */
    bool start();

    /**
     * 停止时隙调度
     */
    void stop();

    /**
     * 处理时隙状态机（需要定期调用）
     */
    void process();

    /**
     * 设置时隙回调函数
     * @param callback 时隙切换时的回调函数
     */
    void setSlotCallback(SlotCallback callback);

    /**
     * 设置同步时间回调函数
     * @param callback 获取同步时间的回调函数
     */
    void setSyncTimeCallback(SyncTimeCallback callback);

    /**
     * 获取当前时隙信息
     * @return 当前时隙信息
     */
    const SlotInfo& getCurrentSlotInfo() const { return currentSlotInfo_; }

    /**
     * 检查是否正在运行
     * @return 是否正在运行
     */
    bool isRunning() const { return isRunning_; }

    /**
     * 获取当前时隙编号
     * @return 当前时隙编号
     */
    uint8_t getCurrentSlot() const { return currentSlotInfo_.currentSlot; }

    /**
     * 检查当前时隙是否为本设备的激活时隙
     * @return 是否为激活时隙
     */
    bool isCurrentSlotActive() const { 
        return currentSlotInfo_.slotType == SlotType::ACTIVE; 
    }

    /**
     * 获取当前激活的引脚编号（仅在激活时隙有效）
     * @return 激活引脚编号，如果非激活时隙返回0xFF
     */
    uint8_t getCurrentActivePin() const;

private:
    // 配置参数
    uint8_t startSlot_;         // 本设备的起始时隙编号
    uint8_t deviceSlotCount_;   // 本设备负责的时隙数量
    uint8_t totalSlotCount_;    // 整个系统的总时隙数
    uint32_t slotIntervalMs_;   // 时隙间隔（毫秒）

    // 运行状态
    bool isRunning_;            // 是否正在运行
    bool isConfigured_;         // 是否已配置
    bool isFirstProcess_;       // 是否是第一次process调用
    uint64_t lastSlotTimeUs_;   // 上次时隙切换时间（微秒）
    uint64_t startTimeUs_;      // 时隙调度开始的绝对时间（微秒）
    SlotInfo currentSlotInfo_;  // 当前时隙信息

    // 回调函数
    SlotCallback slotCallback_;         // 时隙回调
    SyncTimeCallback syncTimeCallback_; // 同步时间回调

    /**
     * 获取当前同步时间（微秒）
     * @return 同步时间
     */
    uint64_t getCurrentSyncTimeUs();

    /**
     * 计算时隙类型和相关信息
     * @param slotNumber 时隙编号
     * @return 时隙类型
     */
    SlotType calculateSlotType(uint8_t slotNumber);

    /**
     * 切换到新时隙
     * @param newSlot 新时隙编号
     */
    void switchToSlot(uint8_t newSlot);
};