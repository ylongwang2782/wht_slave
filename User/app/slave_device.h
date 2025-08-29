#pragma once

#include <memory>

#include "continuity_collector.h"
#include "LockController.h"
#include "slave_device_state.h"
#include "slot_manager.h"
#include "TaskCPP.h"
#include "WhtsProtocol.h"
#include "button.h"
#include "hal_uid.hpp"
namespace SlaveApp {

class IMaster2SlaveMessageHandler;

/**
 * SlaveDevice 类实现了从机设备的功能
 */
class SlaveDevice {
   public:
    // 状态管理
    uint32_t deviceId;
    uint8_t shortId;    // 短ID，由主机分配
    bool isJoined;      // 是否已入网
    bool isConfigured;
    CollectorConfig currentConfig;
    SlaveDeviceState deviceState;

    // 时间同步相关
    int64_t timeOffset;    // 与主机时间的偏移量(us)
    bool isCollecting;     // 是否正在采集数据

    // 延迟启动相关
    uint64_t scheduledStartTime;    // 计划启动时间戳(us)
    bool isScheduledToStart;        // 是否已计划启动

    // 数据发送相关
    std::vector<uint8_t> lastCollectionData;    // 上一次采集的数据，用于延迟发送
    bool hasDataToSend;                         // 是否有数据待发送
    bool isFirstCollection;                     // 是否是第一次采集

    // 待回复消息管理（避免数据冲撞）
    bool hasPendingSlaveControlResponse;        // 是否有待回复的SlaveControl消息
    bool hasPendingResetResponse;               // 是否有待回复的Reset消息
    std::unique_ptr<WhtsProtocol::Message> pendingSlaveControlResponse;  // 待回复的SlaveControl响应
    std::unique_ptr<WhtsProtocol::Message> pendingResetResponse;         // 待回复的Reset响应

    // 设备状态，供外部读取和内部更新
    WhtsProtocol::DeviceStatus deviceStatus;

    WhtsProtocol::ProtocolProcessor processor;

    std::unique_ptr<ContinuityCollector> continuityCollector;
    std::unique_ptr<SlotManager> slotManager;

    static constexpr const char TAG[] = "SlaveDevice";

    SlaveDevice();    // 修改构造函数，自动读取UID
    ~SlaveDevice() = default;

    /**
     * 初始化设备
     * @return 是否成功初始化
     */
    bool initialize();

    /**
     * 运行主循环
     */
    void run();

    /**
     * 处理接收到的帧
     * @param frame 接收到的帧
     */
    void processFrame(WhtsProtocol::Frame& frame);

    /**
     * 发送数据
     * @param frame 要发送的数据帧
     * @return 是否发送成功
     */
    bool send(std::vector<uint8_t>& frame);

    /**
     * 发送待回复的响应消息（在时隙中发送以避免冲撞）
     */
    void sendPendingResponses();

    /**
     * 处理Master2Slave消息
     * @param message 接收到的消息
     * @return 生成的响应消息，如果不需要响应则返回nullptr
     */
    std::unique_ptr<WhtsProtocol::Message> processMaster2SlaveMessage(
        const WhtsProtocol::Message& message);

    /**
     * 设置短ID（由入网流程调用）
     * @param id 分配的短ID
     */
    void setShortId(uint8_t id);

    /**
     * 获取当前时间戳
     * @return 当前时间戳（微秒）
     */
    uint32_t getCurrentTimestamp();

    /**
     * 获取同步时间戳（考虑时间偏移）
     * @return 同步时间戳（微秒）
     */
    uint64_t getSyncTimestampUs();

    /**
     * 获取同步时间戳（考虑时间偏移）
     * @return 同步时间戳（毫秒）
     */
    uint32_t getSyncTimestampMs();

    /**
     * 重置设备状态
     */
    void resetDevice();

    /**
     * 获取当前入网状态
     * @return 是否已入网
     */
    bool getJoinStatus() const { return isJoined; }

    /**
     * 时隙切换回调处理函数
     * @param slotInfo 时隙信息
     */
    void onSlotChanged(const SlotInfo& slotInfo);

   private:
    /**
     * 公告任务类 - 上电时发送AnnounceMsg
     */
    class AnnounceTask : public TaskClassS<512> {
       public:
        AnnounceTask(SlaveDevice& parent);

       private:
        SlaveDevice& parent;
        void task() override;
        bool sendAnnounceMessage();
        static constexpr const char TAG[] = "AnnounceTask";
        static constexpr uint8_t MAX_ANNOUNCE_COUNT = 1;         // 最大发送次数
        static constexpr uint32_t ANNOUNCE_INTERVAL_MS = 500;    // 发送间隔(ms)
    };

    /**
     * 入网管理任务类 - 管理入网流程
     */
    class JoinTask : public TaskClassS<512> {
       public:
        JoinTask(SlaveDevice& parent);

       private:
        SlaveDevice& parent;
        void task() override;
        bool waitForJoin();
        void handleShortIdAssign(uint8_t assignedShortId);
        bool sendShortIdConfirm(uint8_t shortId, uint8_t status);
        static constexpr const char TAG[] = "JoinTask";
        static constexpr uint32_t JOIN_CHECK_INTERVAL_MS =
            100;    // 入网检查间隔
        static constexpr uint32_t JOIN_TIMEOUT_MS =
            10000;    // 入网超时时间(ms)
    };

    /**
     * 数据采集管理任务类 - 管理数据采集状态和处理
     */
    class DataCollectionTask : public TaskClassS<1024> {
       public:
        DataCollectionTask(SlaveDevice& parent);

       public:
        void sendDataToBackend();  // 公开发送数据方法供onSlotChanged调用
        
       private:
        SlaveDevice& parent;
        void task() override;
        void processDataCollection();
        static constexpr const char TAG[] = "DataCollectionTask";
        static constexpr uint32_t PROCESS_INTERVAL_MS = 10;    // 采集处理间隔
    };

    class SlaveDataProcT : public TaskClassS<2048> {
       public:
        SlaveDataProcT(SlaveDevice& parent);

       private:
        std::vector<uint8_t> recvData;
        SlaveDevice& parent;
        void task() override;
        static constexpr const char TAG[] = "SlaveDataProcT";
    };

    /**
     * 外设管理任务类 - 管理按钮、传感器、阀门等外设状态
     */
    class AccessoryTask : public TaskClassS<1024> {
       public:
        AccessoryTask(SlaveDevice& parent);
        void resetLockController();  // 复位锁控制器

       private:
        SlaveDevice& parent;
        void task() override;
        void updateDeviceStatus();
        static constexpr const char TAG[] = "AccessoryTask";
        static constexpr uint32_t UPDATE_INTERVAL_MS = 300;    // 状态更新间隔
        static constexpr uint32_t BATTERY_UPDATE_INTERVAL_MS = 1000;    // 电池电压更新间隔
        static constexpr float BATTERY_VOLTAGE_THRESHOLD = 3.50f;    // 电池电压阈值
        static constexpr float BATTERY_VOLTAGE_THRESHOLD_OFFSET = 0.05f;    // 电池电压阈值偏移

        HalButton key1;
        HalButton unlockBtn;
        HalButton auxBtn1;
        HalButton auxBtn2;
        HalSensor pSensor;
        HalSensor clrSensor;
        HalValve valve1;
        DipSwitchInfo dipInfo;
        HalDipSwitch dipSwitch;
        LockController lockController;

        uint16_t adc_value;
        float battery_voltage;
    };

    std::unique_ptr<DataCollectionTask> dataCollectionTask;
    std::unique_ptr<AnnounceTask> announceTask;
    std::unique_ptr<JoinTask> joinTask;
    std::unique_ptr<SlaveDataProcT> slaveDataProcT;
    std::unique_ptr<AccessoryTask> accessoryTask;

    // Message handlers array for O(1) lookup
    IMaster2SlaveMessageHandler* messageHandlers_[256] = {};

    // Initialize message handlers
    void initializeMessageHandlers();
};

}    // namespace SlaveApp