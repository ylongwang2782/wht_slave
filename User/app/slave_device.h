#pragma once

#include <memory>

#include "LockController.h"
#include "MasterComm.h"
#include "TaskCPP.h"
#include "WhtsProtocol.h"
#include "button.h"
#include "continuity_collector.h"
#include "slave_device_state.h"
#include "slot_manager.h"

namespace SlaveApp
{

// 采集模式枚举
enum class CollectionMode : uint8_t
{
    CONDUCTION = 0, // 导通检测
    RESISTANCE = 1, // 阻值检测
    CLIP = 2        // 卡钉检测
};

// 从机配置结构体（扩展原有的CollectorConfig）
struct SlaveDeviceConfig
{
    CollectionMode mode; // 采集模式
    uint8_t interval;    // 采集间隔（ms）
    uint8_t timeSlot;    // 分配的时隙
    uint8_t testCount;   // 检测数量

    SlaveDeviceConfig() : mode(CollectionMode::CONDUCTION), interval(100), timeSlot(0), testCount(2)
    {
    }
};

class IMaster2SlaveMessageHandler;

/**
 * SlaveDevice 类实现了从机设备的功能
 */
class SlaveDevice
{
  public:
    // 状态管理
    uint32_t m_deviceId;
    uint8_t m_shortId;               // 短ID，由主机分配
    bool m_isJoined;                 // 是否已入网
    bool m_isConfigured;             // 是否已配置
    SlaveDeviceConfig currentConfig; // 当前配置
    SlaveDeviceState m_deviceState;  // 设备状态

    // 时间同步相关
    int64_t m_timeOffset; // 与主机时间的偏移量(us)
    bool m_isCollecting;  // 是否正在采集数据

    // 心跳相关
    uint64_t m_lastSyncMessageTime;                                // 上次收到sync消息的时间戳(us)
    uint64_t m_lastHeartbeatTime;                                  // 上次发送心跳的时间戳(us)
    bool m_inTdmaMode;                                             // 是否在TDMA管理模式下
    static constexpr uint64_t SYNC_TIMEOUT_US = 30000000ULL;       // 30秒超时(us)
    static constexpr uint64_t HEARTBEAT_INTERVAL_US = 10000000ULL; // 10秒心跳间隔(us)

    // 延迟启动相关
    uint64_t m_scheduledStartTime; // 计划启动时间戳(us)
    bool m_isScheduledToStart;     // 是否已计划启动

    // 数据发送相关
    std::vector<uint8_t> lastCollectionData; // 上一次采集的数据，用于延迟发送
    bool m_hasDataToSend;                    // 是否有数据待发送
    bool m_isFirstCollection;                // 是否是第一次采集

    // 待回复消息管理（避免数据冲撞）
    bool m_hasPendingSlaveControlResponse;                                // 是否有待回复的SlaveControl消息
    bool m_hasPendingResetResponse;                                       // 是否有待回复的Reset消息
    std::unique_ptr<WhtsProtocol::Message> m_pendingSlaveControlResponse; // 待回复的SlaveControl响应
    std::unique_ptr<WhtsProtocol::Message> m_pendingResetResponse;        // 待回复的Reset响应

    // 设备状态，供外部读取和内部更新
    WhtsProtocol::DeviceStatus m_deviceStatus;

    WhtsProtocol::ProtocolProcessor m_processor;

    std::unique_ptr<ContinuityCollector> m_continuityCollector;
    std::unique_ptr<SlotManager> m_slotManager;

    static constexpr const char TAG[] = "SlaveDevice";

    SlaveDevice();
    ~SlaveDevice() = default;

    /**
     * 运行主循环
     */
    void run() const;

    /**
     * 处理接收到的帧
     * @param frame 接收到的帧
     */
    void processFrame(const WhtsProtocol::Frame &frame);

    /**
     * 发送数据
     * @param frame 要发送的数据帧
     * @return 是否发送成功
     */
    int send(const std::vector<uint8_t> &frame);

    /**
     * 发送待回复的响应消息（在时隙中发送以避免冲撞）
     */
    void sendPendingResponses();

    /**
     * 发送心跳消息
     */
    void sendHeartbeat();

    /**
     * 发送公告消息
     */
    void sendJoinRequestMessage();

    /**
     * 计算电池电量百分比
     * @param voltage 电池电压 (V)
     * @return 电池电量百分比 (0-100%)
     */
    static uint8_t calculateBatteryPercentage(float voltage);

    /**
     * 获取当前电池电量百分比
     * @return 电池电量百分比 (0-100%)
     */
    uint8_t getCurrentBatteryPercentage();

    /**
     * 处理Master2Slave消息
     * @param message 接收到的消息
     * @return 生成的响应消息，如果不需要响应则返回nullptr
     */
    std::unique_ptr<WhtsProtocol::Message> processMaster2SlaveMessage(const WhtsProtocol::Message &message);

    /**
     * 设置短ID（由入网流程调用）
     * @param id 分配的短ID
     */
    void setShortId(uint8_t id);

    /**
     * 获取当前时间戳
     * @return 当前时间戳（微秒）
     */
    static uint32_t getCurrentTimestamp();

    /**
     * 获取同步时间戳（考虑时间偏移）
     * @return 同步时间戳（微秒）
     */
    [[nodiscard]] uint64_t GetSyncTimestampUs() const;

    /**
     * 获取同步时间戳（考虑时间偏移）
     * @return 同步时间戳（毫秒）
     */
    [[nodiscard]] uint32_t GetSyncTimestampMs() const;

    /**
     * 重置设备状态
     */
    void resetDevice() const;

    /**
     * 获取当前入网状态
     * @return 是否已入网
     */
    [[nodiscard]] bool getJoinStatus() const
    {
        return m_isJoined;
    }

    /**
     * 时隙切换回调处理函数
     * @param slotInfo 时隙信息
     */
    void OnSlotChanged(const SlotInfo &slotInfo);

  private:
    /**
     * 数据采集管理任务类 - 管理数据采集状态和处理
     */
    class DataCollectionTask final : public TaskClassS<1024>
    {
      public:
        explicit DataCollectionTask(SlaveDevice &parent);

      public:
        void sendDataToBackend() const; // 公开发送数据方法供onSlotChanged调用

      private:
        SlaveDevice &parent;
        void task() override;
        void processDataCollection() const;
        static constexpr const char TAG[] = "DataCollectionTask";
        static constexpr uint32_t PROCESS_INTERVAL_MS = 10; // 采集处理间隔
    };

    class SlaveDataProcT final : public TaskClassS<2048>
    {
      public:
        explicit SlaveDataProcT(SlaveDevice &parent);

      private:
        std::vector<uint8_t> recvData;
        SlaveDevice &parent;
        void task() override;
        static constexpr const char TAG[] = "SlaveDataProcT";
    };

    /**
     * 外设管理任务类 - 管理按钮、传感器、阀门等外设状态
     */
    class AccessoryTask final : public TaskClassS<1024>
    {
      public:
        explicit AccessoryTask(SlaveDevice &parent);
        void resetLockController(); // 复位锁控制器

      private:
        SlaveDevice &parent;
        void task() override;
        void updateDeviceStatus();
        static constexpr const char TAG[] = "AccessoryTask";
        static constexpr uint32_t UPDATE_INTERVAL_MS = 300;              // 状态更新间隔
        static constexpr uint32_t BATTERY_UPDATE_INTERVAL_MS = 1000;     // 电池电压更新间隔
        static constexpr float BATTERY_VOLTAGE_THRESHOLD = 3.50f;        // 电池电压阈值
        static constexpr float BATTERY_VOLTAGE_THRESHOLD_OFFSET = 0.05f; // 电池电压阈值偏移

        HalButton key1;
        HalButton unlockBtn;
        HalButton auxBtn1;
        HalButton auxBtn2;
        HalSensor pSensor;
        HalSensor clrSensor;
        HalValve valve1;
        HalValve valve2;
        HalValve valve3;
        HalValve valve4;
        DipSwitchInfo dipInfo;
        HalDipSwitch dipSwitch;
        LockController lockController;

        uint16_t adc_value{};
        float battery_voltage{};
    };

    MasterComm m_masterComm;

    std::unique_ptr<DataCollectionTask> m_dataCollectionTask;
    std::unique_ptr<SlaveDataProcT> m_slaveDataProcT;
    std::unique_ptr<AccessoryTask> m_accessoryTask;

    // Message handlers array for O(1) lookup
    IMaster2SlaveMessageHandler *messageHandlers_[256] = {};

    // Initialize message handlers
    void InitializeMessageHandlers();
};

} // namespace SlaveApp