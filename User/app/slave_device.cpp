
#include "slave_device.h"

#include <cstdio>

#include "MasterComm.h"
#include "adc.h"
#include "cmsis_os2.h"
#include "elog.h"
#include "hal_uid.hpp"
#include "hptimer.hpp"
#include "main.h"
#include "master_slave_message_handlers.h"

using namespace WhtsProtocol;

// 发送数据入队超时
#define MsgProc_TX_QUEUE_TIMEOUT 1000
// 发送超时
#define MsgProc_TX_TIMEOUT 1000

namespace SlaveApp
{

SlaveDevice::SlaveDevice()
    : m_deviceId(DeviceUID::get()),                                                  // 自动读取设备UID
      m_shortId(0),                                                                  // 初始短ID为0，表示未分配
      m_isJoined(false),                                                             // 初始未入网
      m_isConfigured(false), m_deviceState(SlaveDeviceState::IDLE), m_timeOffset(0), // 初始时间偏移量为0
      m_isCollecting(false),                                                         // 初始未在采集
      m_lastSyncMessageTime(0),                                                      // 初始化上次sync消息时间
      m_lastHeartbeatTime(HptimerGetUs()),                                           // 初始化上次心跳时间为当前时间
      m_inTdmaMode(false),                                                           // 初始不在TDMA模式
      m_scheduledStartTime(0),                                                       // 初始计划启动时间为0
      m_isScheduledToStart(false),                                                   // 初始未计划启动
      m_hasDataToSend(false),                                                        // 初始无数据待发送
      m_isFirstCollection(true),                                                     // 初始为第一次采集
      m_hasPendingSlaveControlResponse(false),                                       // 初始无待回复的SlaveControl消息
      m_hasPendingResetResponse(false),                                              // 初始无待回复的Reset消息
      m_pendingSlaveControlResponse(nullptr), // 初始化待回复的SlaveControl响应为空
      m_pendingResetResponse(nullptr),        // 初始化待回复的Reset响应为空
      m_deviceStatus({}), m_masterComm()
{

    // Initialize continuity collector
    m_continuityCollector = ContinuityCollectorFactory::Create();

    // Initialize slot manager
    m_slotManager = std::make_unique<SlotManager>();

    // 设置时隙管理器的同步时间回调
    if (m_slotManager)
    {
        m_slotManager->SetSyncTimeCallback([this]() { return this->GetSyncTimestampUs(); });

        // 设置时隙切换回调
        m_slotManager->SetSlotCallback([this](const SlotInfo &slotInfo) { this->OnSlotChanged(slotInfo); });
    }

    // set MTU = 1016
    m_processor.SetMTU(800);

    // Initialize message handlers
    InitializeMessageHandlers();

    m_dataCollectionTask = std::make_unique<DataCollectionTask>(*this);
    m_slaveDataProcT = std::make_unique<SlaveDataProcT>(*this);
    m_accessoryTask = std::make_unique<AccessoryTask>(*this);
}

void SlaveDevice::InitializeMessageHandlers()
{
    messageHandlers_[static_cast<uint8_t>(WhtsProtocol::Master2SlaveMessageId::SYNC_MSG)] =
        &SyncMessageHandler::GetInstance();
    messageHandlers_[static_cast<uint8_t>(WhtsProtocol::Master2SlaveMessageId::PING_REQ_MSG)] =
        &PingRequestHandler::GetInstance();
    messageHandlers_[static_cast<uint8_t>(WhtsProtocol::Master2SlaveMessageId::SHORT_ID_ASSIGN_MSG)] =
        &ShortIdAssignHandler::GetInstance();
}

std::unique_ptr<Message> SlaveDevice::processMaster2SlaveMessage(const Message &message)
{
    elog_v("SlaveDevice", "Processing Master2Slave message: %s", message.getMessageTypeName());

    const uint8_t messageId = message.getMessageId();
    elog_v("SlaveDevice", "Processing Master2Slave message, ID: 0x%02X", static_cast<int>(messageId));

    if (IMaster2SlaveMessageHandler *handler = messageHandlers_[messageId])
    {
        // Process message using the appropriate handler
        auto response = handler->ProcessMessage(message, this);

        if (response)
        {
            elog_v("SlaveDevice", "Response generated for Master2Slave message");
        }
        else
        {
            elog_v("SlaveDevice", "No response needed for this Master2Slave message");
        }

        return response;
    }
    else
    {
        elog_w("SlaveDevice", "Unknown Master2Slave message type: 0x%02X", static_cast<int>(messageId));
        return nullptr;
    }
}

uint32_t SlaveDevice::getCurrentTimestamp()
{
    const uint32_t tick = HptimerGetUs();
    return tick;
}

uint8_t SlaveDevice::calculateBatteryPercentage(float voltage)
{
    // 电池电压范围: 3.2V (0%) - 4.2V (100%)
    constexpr float MIN_VOLTAGE = 3.2f;
    constexpr float MAX_VOLTAGE = 4.2f;

    // 限制电压范围
    if (voltage <= MIN_VOLTAGE)
    {
        return 0;
    }
    if (voltage >= MAX_VOLTAGE)
    {
        return 100;
    }

    // 线性计算百分比
    float percentage = ((voltage - MIN_VOLTAGE) / (MAX_VOLTAGE - MIN_VOLTAGE)) * 100.0f;
    return static_cast<uint8_t>(percentage + 0.5f); // 四舍五入
}

uint8_t SlaveDevice::getCurrentBatteryPercentage()
{
    // 读取ADC值获取电池电压
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    uint16_t adc_value = HAL_ADC_GetValue(&hadc1);

    // 转换为电压值 (参考AccessoryTask中的计算方式)
    float battery_voltage = (adc_value / 4095.0f) * 3.15f * 2.0f;

    // 计算电池电量百分比
    return calculateBatteryPercentage(battery_voltage);
}

uint64_t SlaveDevice::GetSyncTimestampUs() const
{
    // 获取当前本地时间（微秒）
    const uint64_t localTimeUs = HptimerGetUs();

    // 应用时间偏移量得到同步时间（微秒）
    const uint64_t syncTimeUs = localTimeUs + m_timeOffset;

    // 转换为毫秒
    return syncTimeUs;
}

uint32_t SlaveDevice::GetSyncTimestampMs() const
{
    // 获取当前本地时间（微秒）
    const uint64_t localTimeUs = HptimerGetUs();

    // 应用时间偏移量得到同步时间（微秒）
    const uint64_t syncTimeUs = localTimeUs + m_timeOffset;

    // 转换为毫秒
    return static_cast<uint32_t>(syncTimeUs / 1000);
}

void SlaveDevice::resetDevice() const
{
    // reset Device only reset lockController1 for now design
    if (m_accessoryTask)
    {
        m_accessoryTask->resetLockController();
    }

    elog_v("SlaveDevice", "Device reset to READY state, configuration preserved");
}

void SlaveDevice::sendPendingResponses()
{
    // 发送待回复的Reset响应
    if (m_hasPendingResetResponse && m_pendingResetResponse)
    {
        elog_v(TAG, "Sending pending Reset response in active slot");

        std::vector<std::vector<uint8_t>> responseData =
            m_processor.packSlave2MasterMessage(m_deviceId, *m_pendingResetResponse);

        // 发送所有片段
        bool success = true;
        for (auto &fragment : responseData)
        {
            if (send(fragment) != 0)
            {
                elog_e(TAG, "Failed to send Reset response fragment");
                success = false;
                break;
            }
        }

        if (success)
        {
            elog_v(TAG, "Reset response sent successfully");
        }

        // 清除待回复状态
        m_hasPendingResetResponse = false;
        m_pendingResetResponse.reset();
    }

    // 发送待回复的SlaveControl响应
    if (m_hasPendingSlaveControlResponse && m_pendingSlaveControlResponse)
    {
        elog_v(TAG, "Sending pending SlaveControl response in active slot");

        std::vector<std::vector<uint8_t>> responseData =
            m_processor.packSlave2MasterMessage(m_deviceId, *m_pendingSlaveControlResponse);

        // 发送所有片段
        bool success = true;
        for (auto &fragment : responseData)
        {
            if (send(fragment) != 0)
            {
                elog_e(TAG, "Failed to send SlaveControl response fragment");
                success = false;
                break;
            }
        }

        if (success)
        {
            elog_v(TAG, "SlaveControl response sent successfully");
        }

        // 清除待回复状态
        m_hasPendingSlaveControlResponse = false;
        m_pendingSlaveControlResponse.reset();
    }
}

void SlaveDevice::sendHeartbeat()
{
    // 获取当前电池电量百分比
    uint8_t batteryPercentage = getCurrentBatteryPercentage();

    elog_v(TAG, "Sending heartbeat message with battery level: %d%%", batteryPercentage);

    // 创建心跳消息
    auto heartbeatMsg = std::make_unique<WhtsProtocol::Slave2Master::HeartbeatMessage>();
    heartbeatMsg->batteryLevel = batteryPercentage;

    // 打包消息
    std::vector<std::vector<uint8_t>> messageData = m_processor.packSlave2MasterMessage(m_deviceId, *heartbeatMsg);

    // 发送所有片段
    bool success = true;
    for (auto &fragment : messageData)
    {
        if (send(fragment) != 0)
        {
            elog_e(TAG, "Failed to send heartbeat fragment");
            success = false;
            break;
        }
    }

    if (success)
    {
        elog_v(TAG, "Heartbeat sent successfully with battery level: %d%%", batteryPercentage);
        m_lastHeartbeatTime = HptimerGetUs();
    }
    else
    {
        elog_e(TAG, "Failed to send heartbeat");
    }
}

void SlaveDevice::sendJoinRequestMessage()
{
    elog_v(TAG, "Sending joinRequest message");

    // 创建公告消息
    auto joinRequestMsg = std::make_unique<WhtsProtocol::Slave2Master::JoinRequestMessage>();
    joinRequestMsg->deviceId = m_deviceId;
    joinRequestMsg->versionMajor = FIRMWARE_VERSION_MAJOR;
    joinRequestMsg->versionMinor = FIRMWARE_VERSION_MINOR;
    joinRequestMsg->versionPatch = FIRMWARE_VERSION_PATCH;

    // 打包消息
    std::vector<std::vector<uint8_t>> messageData = m_processor.packSlave2MasterMessage(m_deviceId, *joinRequestMsg);

    // 发送所有片段
    bool success = true;
    for (auto &fragment : messageData)
    {
        if (send(fragment) != 0)
        {
            elog_e(TAG, "Failed to send joinRequest message fragment");
            success = false;
            break;
        }
    }

    if (success)
    {
        elog_v(TAG, "JoinRequest message sent successfully");
    }
    else
    {
        elog_e(TAG, "Failed to send joinRequest message");
    }
}

void SlaveDevice::OnSlotChanged(const SlotInfo &slotInfo)
{
    // 如果有待回复的响应，即使不在采集状态也要处理
    if (!m_isCollecting && (m_hasPendingResetResponse || m_hasPendingSlaveControlResponse))
    {
        if (slotInfo.m_slotType == SlotType::ACTIVE && slotInfo.m_activePin == 0)
        {
            elog_v(TAG, "Sending pending responses even when not collecting");
            sendPendingResponses();
        }
        return;
    }

    // 只在采集状态下处理采集相关的时隙事件
    if (!m_isCollecting || !m_continuityCollector || !m_slotManager)
    {
        return;
    }

    // 检查是否是本设备的第一个激活时隙
    if (slotInfo.m_slotType == SlotType::ACTIVE && slotInfo.m_activePin == 0)
    { // 第一个激活引脚

        // 优先发送待回复的响应消息（避免与数据传输冲撞）
        sendPendingResponses();

        // 在TDMA模式下发送相应消息（当前在第一个激活时隙）
        if (m_inTdmaMode)
        {
            // 检查是否已分配短ID
            if (m_isJoined && m_shortId != 0)
            {
                // 有短ID，发送心跳包
                sendHeartbeat();
            }
        }

        // 发送数据到后端（每个周期都发送当前采集的数据）
        if (m_dataCollectionTask)
        {
            elog_v(TAG, "Sending current collection data to backend");
            m_dataCollectionTask->sendDataToBackend();
        }
    }

    // 通知采集器处理当前时隙
    m_continuityCollector->ProcessSlot(slotInfo.m_currentSlot, slotInfo.m_activePin,
                                       slotInfo.m_slotType == SlotType::ACTIVE);

    // 检查采集是否完成
    if (m_continuityCollector->IsCollectionComplete())
    {
        elog_v(TAG, "Data collection cycle completed, saving data for next cycle");

        // 保存当前采集的数据，供下一个周期发送
        const auto dataVector = m_continuityCollector->GetDataVector();
        lastCollectionData = dataVector;
        m_hasDataToSend = true;
        elog_v(TAG, "Saved %d bytes of data for next cycle transmission", dataVector.size());

        // 清空数据矩阵为下一次采集做准备
        m_continuityCollector->ClearData();
    }
}

void SlaveDevice::setShortId(const uint8_t id)
{
    m_shortId = id;
    m_isJoined = true;
    elog_d(TAG, "Short ID assigned: %d, device joined successfully", m_shortId);
}

void SlaveDevice::processFrame(const Frame &frame)
{
    elog_v("SlaveDevice", "Processing frame - PacketId: 0x%02X, payload size: %d", static_cast<int>(frame.packetId),
           frame.payload.size());

    if (frame.packetId == static_cast<uint8_t>(PacketId::MASTER_TO_SLAVE))
    {
        std::unique_ptr<Message> masterMessage;

        if (uint32_t targetSlaveId; m_processor.parseMaster2SlavePacket(frame.payload, targetSlaveId, masterMessage))
        {
            // Check if this message is for us (or broadcast)
            if (targetSlaveId == m_deviceId || targetSlaveId == BROADCAST_ID)
            {
                elog_d("SlaveDevice",
                       "Received Master2Slave message for device 0x%08X, "
                       "Message: %s (ID: 0x%02X)",
                       targetSlaveId, masterMessage->getMessageTypeName(),
                       static_cast<int>(masterMessage->getMessageId()));

                // Process message and create response

                if (const auto response = processMaster2SlaveMessage(*masterMessage))
                {
                    elog_v("SlaveDevice", "Generated response message");

                    elog_v("SlaveDevice", "Packing Slave2Master message: %s", response->getMessageTypeName());
                    std::vector<std::vector<uint8_t>> responseData =
                        m_processor.packSlave2MasterMessage(m_deviceId, *response);

                    elog_v("SlaveDevice", "Sending response:");

                    // Send all fragments to master
                    for (auto &fragment : responseData)
                    {
                        if (send(fragment) != 0)
                        {
                            elog_e("SlaveDevice", "Failed to send response fragment");
                        }
                    }
                }
            }
            else
            {
                elog_v("SlaveDevice",
                       "Message not for this device (target: 0x%08X, our ID: "
                       "0x%08X)",
                       targetSlaveId, m_deviceId);
            }
        }
        else
        {
            elog_e("SlaveDevice", "Failed to parse Master2Slave packet");
        }
    }
    else
    {
        // elog_w("SlaveDevice", "Unsupported packet type for Slave: 0x%02X",
        //        static_cast<int>(frame.packetId));
    }
}

void SlaveDevice::run() const
{
    if (m_dataCollectionTask)
    {
        m_dataCollectionTask->give();
        elog_d(TAG, "DataCollectionTask initialized and started");
    }

    if (m_slaveDataProcT)
    {
        m_slaveDataProcT->give();
        elog_d(TAG, "SlaveDataProcT initialized and started");
    }

    if (m_accessoryTask)
    {
        m_accessoryTask->give();
        elog_d(TAG, "AccessoryTask initialized and started");
    }

    while (true)
    {
        HAL_GPIO_TogglePin(RUN_LED_GPIO_Port, RUN_LED_Pin);
        HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
        TaskBase::delay(500);
    }
}

// DataCollectionTask 实现
SlaveDevice::DataCollectionTask::DataCollectionTask(SlaveDevice &parent)
    : TaskClassS("DataCollectionTask", TaskPrio_Mid), parent(parent)
{
}

void SlaveDevice::DataCollectionTask::task()
{
    elog_d(TAG, "Data collection task started");

    bool wasSlotManagerRunning = false;

    for (;;)
    {
        // 处理数据采集状态
        processDataCollection();

        // 处理时隙管理器状态（只要SlotManager在运行就需要处理）
        if (parent.m_slotManager)
        {
            bool isCurrentlyRunning = parent.m_slotManager->IsRunning();

            // 检测SlotManager从运行状态变为停止状态
            if (wasSlotManagerRunning && !isCurrentlyRunning && parent.m_isCollecting)
            {
                elog_v(TAG, "SlotManager stopped, single cycle completed");
                parent.m_isCollecting = false;
                parent.m_deviceState = SlaveDeviceState::IDLE;
                parent.m_isScheduledToStart = false;
                parent.m_scheduledStartTime = 0;
            }

            wasSlotManagerRunning = isCurrentlyRunning;

            parent.m_slotManager->Process();
        }

        // 减少轮询间隔以提高时隙切换精度
        TaskBase::delay(1);
    }
}

void SlaveDevice::DataCollectionTask::processDataCollection() const
{
    // 检查是否有计划启动的采集
    if (parent.m_isScheduledToStart && !parent.m_isCollecting)
    {
        uint64_t currentTimeUs = parent.GetSyncTimestampUs();

        // 检查是否到达启动时间
        if (currentTimeUs >= parent.m_scheduledStartTime)
        {
            elog_v(TAG,
                   "Scheduled start time reached, starting data collection. "
                   "Current: %lu us, Scheduled: %lu us",
                   static_cast<unsigned long>(currentTimeUs), static_cast<unsigned long>(parent.m_scheduledStartTime));

            // 配置和启动采集器
            bool collectorReady = true;
            if (parent.m_continuityCollector && parent.m_slotManager)
            {
                // 从SlotManager获取总时隙数作为总周期数
                uint16_t totalCycles = parent.m_slotManager->GetTotalSlots();
                CollectorConfig collectorConfig(parent.currentConfig.testCount, totalCycles);
                if (!parent.m_continuityCollector->Configure(collectorConfig))
                {
                    elog_e(TAG, "Failed to configure continuity collector for scheduled start");
                    collectorReady = false;
                }
                else
                {
                    elog_v(TAG, "Continuity collector configured for delayed start - TestCount: %d, TotalCycles: %d",
                           parent.currentConfig.testCount, totalCycles);
                }
            }

            // 启动采集和时隙管理器
            if (collectorReady && parent.m_continuityCollector && parent.m_slotManager &&
                parent.m_continuityCollector->StartCollection() && parent.m_slotManager->Start())
            {
                parent.m_isCollecting = true;
                parent.m_deviceState = SlaveDeviceState::RUNNING;
                parent.m_isScheduledToStart = false;
                parent.m_scheduledStartTime = 0;
                parent.m_isFirstCollection = true; // 重置为第一次采集
                elog_v(TAG, "Data collection and slot management started "
                            "successfully from scheduled start");
            }
            else
            {
                elog_e(TAG, "Failed to start scheduled data collection or slot "
                            "management");
                parent.m_deviceState = SlaveDeviceState::DEV_ERR;
                parent.m_isScheduledToStart = false;
                parent.m_scheduledStartTime = 0;
            }
        }
    }

    // processDataCollection方法现在主要负责启动调度
    // 实际的采集和数据处理由时隙管理器通过onSlotChanged回调处理
}

void SlaveDevice::DataCollectionTask::sendDataToBackend() const
{
    if (!parent.m_isConfigured)
    {
        elog_w(TAG, "Device not configured");
        return;
    }

    // 检查是否有缓存的数据可发送
    if (!parent.m_hasDataToSend || parent.lastCollectionData.empty())
    {
        elog_w(TAG, "No cached data available to send to backend");
        return;
    }

    // 根据当前配置创建相应的数据消息
    // 这里假设是导通检测模式，实际应该根据配置的模式来决定
    auto dataMsg = std::make_unique<Slave2Backend::ConductionDataMessage>();

    // 使用缓存的数据
    dataMsg->conductionData = parent.lastCollectionData;
    dataMsg->conductionLength = dataMsg->conductionData.size();

    if (dataMsg->conductionLength > 0)
    {
        elog_v(TAG, "Sending %d bytes of cached conduction data to backend", dataMsg->conductionLength);

        // 使用协议处理器打包消息为Slave2Backend格式
        const auto packedData =
            parent.m_processor.packSlave2BackendMessage(parent.m_deviceId, parent.m_deviceStatus, *dataMsg);

        // 发送所有数据包片段
        bool success = true;
        for (size_t i = 0; i < packedData.size(); ++i)
        {
            if (parent.send(packedData[i]) != 0)
            {
                elog_e(TAG, "Failed to send data fragment %d/%d to backend", i + 1, packedData.size());
                success = false;
                break;
            }
        }

        if (success)
        {
            elog_v(TAG, "Cached data successfully sent to backend (%d fragments)", packedData.size());
            // 发送成功后清空缓存数据和标志
            parent.lastCollectionData.clear();
            parent.m_hasDataToSend = false;
        }
        else
        {
            elog_e(TAG, "Failed to send complete cached data to backend");
        }
    }
    else
    {
        elog_w(TAG, "No cached data available to send to backend");
    }
}

int SlaveDevice::send(const std::vector<uint8_t> &frame)
{
    return m_masterComm.SendData(frame.data(), frame.size(), 0);
}

// SlaveDataProcT 实现
SlaveDevice::SlaveDataProcT::SlaveDataProcT(SlaveDevice &parent)
    : TaskClassS("SlaveDataProcT", TaskPrio_Mid), parent(parent)
{
}

void SlaveDevice::SlaveDataProcT::task()
{
    elog_v(TAG, "SlaveDataProcT started");
    uwbRxMsg msg;
    for (;;)
    {
        if (parent.m_masterComm.ReceiveData(&msg, 0) == 0)
        {
            elog_v(TAG, "SlaveDataProcT recvData size: %d", msg.dataLen);
            // copy msg.data to recvData
            recvData.assign(msg.data, msg.data + msg.dataLen);

            if (!recvData.empty())
            {
                // process recvData
                elog_v(TAG, "recvData size: %d", recvData.size());
                parent.m_processor.processReceivedData(recvData);

                // process complete frame
                Frame receivedFrame;
                while (parent.m_processor.getNextCompleteFrame(receivedFrame))
                {
                    parent.processFrame(receivedFrame);
                }
                recvData.clear();
            }
        }
        TaskBase::delay(1);
    }
}

// AccessoryTask 实现
SlaveDevice::AccessoryTask::AccessoryTask(SlaveDevice &parent)
    : TaskClassS("AccessoryTask", TaskPrio_Mid), parent(parent), key1("Key1", KEY1_GPIO_Port, KEY1_Pin),
      unlockBtn("unlockBtn", UNLOCK_BTN_GPIO_Port, UNLOCK_BTN_Pin),
      auxBtn1("auxBtn1", AUX_BTN1_GPIO_Port, AUX_BTN1_Pin), auxBtn2("auxBtn2", AUX_BTN2_GPIO_Port, AUX_BTN2_Pin),
      pSensor("pSensor", P_SENSOR_GPIO_Port, P_SENSOR_Pin),
      clrSensor("clrSensor", CLR_SENSOR_GPIO_Port, CLR_SENSOR_Pin), valve1("Valve1", ELV1_GPIO_Port, ELV1_Pin),
      valve2("Valve2", ELV2_GPIO_Port, ELV2_Pin), valve3("Valve3", ELV3_GPIO_Port, ELV3_Pin),
      valve4("Valve4", ELV4_GPIO_Port, ELV4_Pin), dipInfo{{
                                                      {DIP1_GPIO_Port, DIP1_Pin},
                                                      {DIP2_GPIO_Port, DIP2_Pin},
                                                      {DIP3_GPIO_Port, DIP3_Pin},
                                                      {DIP4_GPIO_Port, DIP4_Pin},
                                                      {DIP5_GPIO_Port, DIP5_Pin},
                                                      {DIP6_GPIO_Port, DIP6_Pin},
                                                      {DIP7_GPIO_Port, DIP7_Pin},
                                                      {DIP8_GPIO_Port, DIP8_Pin},
                                                  }},
      dipSwitch(dipInfo),
      lockController("LockController", key1, unlockBtn, std::vector<HalValve *>{&valve1, &valve2, &valve3, &valve4})

{
}

void SlaveDevice::AccessoryTask::task()
{
    elog_d(TAG, "Accessory hardware initialized");

    // send joinRequest when first power on
    parent.sendJoinRequestMessage();

    for (;;)
    {

        // 检查sync消息超时和心跳逻辑
        uint64_t currentTime = HptimerGetUs();

        // 检查是否超过30秒没收到sync消息（退出TDMA模式）
        if (parent.m_inTdmaMode && (currentTime - parent.m_lastSyncMessageTime) > parent.SYNC_TIMEOUT_US)
        {
            elog_v(TAG, "Sync message timeout, exiting TDMA mode");
            parent.m_inTdmaMode = false;
            parent.m_lastHeartbeatTime = currentTime; // 重置心跳计时
        }

        // 在非TDMA模式下，每10秒检测短ID状态并发送相应消息
        if (!parent.m_inTdmaMode && (currentTime - parent.m_lastHeartbeatTime) > parent.HEARTBEAT_INTERVAL_US)
        {
            // 检查是否已分配短ID
            if (parent.m_isJoined && parent.m_shortId != 0)
            {
                // 有短ID，发送心跳包
                elog_v(TAG, "Sending periodic heartbeat outside TDMA mode (Short ID: %d)", parent.m_shortId);
                parent.sendHeartbeat();
            }
            else
            {
                // 没有短ID，发送公告消息
                elog_v(TAG, "Sending periodic joinRequest message (no Short ID assigned)");
                parent.sendJoinRequestMessage();
            }
        }

        // 自动处理逻辑
        lockController.Update();

        // 更新设备状态
        updateDeviceStatus();

        osDelay(UPDATE_INTERVAL_MS);
    }
}

void SlaveDevice::AccessoryTask::resetLockController()
{
    lockController.Reset();
}

void SlaveDevice::AccessoryTask::updateDeviceStatus()
{
    parent.m_deviceStatus.pressureSensor = pSensor.IsTrigger();
    parent.m_deviceStatus.clrSensor = clrSensor.IsTrigger();
    parent.m_deviceStatus.accessory1 = auxBtn1.IsPressed();
    parent.m_deviceStatus.accessory2 = auxBtn2.IsPressed();
    parent.m_deviceStatus.electromagneticLock1 = (lockController.GetState() == LockState::Locked);
    parent.m_deviceStatus.electromagneticLock2 = false;
    parent.m_deviceStatus.electromagnetUnlockButton = unlockBtn.IsPressed();
    parent.m_deviceStatus.sleeveLimit = false;

    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    adc_value = HAL_ADC_GetValue(&hadc1);
    battery_voltage = (adc_value / 4095.0f) * 3.15f * 2.0f;

    // 判断逻辑（带滞回）
    if (!parent.m_deviceStatus.batteryLowAlarm && battery_voltage < BATTERY_VOLTAGE_THRESHOLD)
    {
        parent.m_deviceStatus.batteryLowAlarm = true;
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
    }
    else if (parent.m_deviceStatus.batteryLowAlarm &&
             battery_voltage > BATTERY_VOLTAGE_THRESHOLD + BATTERY_VOLTAGE_THRESHOLD_OFFSET)
    {
        parent.m_deviceStatus.batteryLowAlarm = false;
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
    }
}

} // namespace SlaveApp