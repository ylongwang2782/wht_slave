#include "master_slave_message_handlers.h"

#include "elog.h"
#include "hptimer.hpp"
#include "slave_device.h"

using namespace WhtsProtocol;

namespace SlaveApp
{

// Sync Message Handler
std::unique_ptr<Message> SyncMessageHandler::ProcessMessage(const Message &message, SlaveDevice *device)
{
    auto syncMsg = dynamic_cast<const Master2Slave::SyncMessage *>(&message);
    if (!syncMsg)
        return nullptr;

    elog_v("SyncMessageHandler", "Processing sync message - Timestamp: %lu us",
           static_cast<unsigned long>(syncMsg->timestamp));

    // 进行时间校准，计算与主机时间的偏移量
    uint64_t localTimestamp = HptimerGetUs();
    int64_t timeOffset = static_cast<int64_t>(syncMsg->timestamp) - static_cast<int64_t>(localTimestamp);

    // 存储时间偏移量用于后续采集任务
    device->m_timeOffset = timeOffset;

    elog_i("SyncMessageHandler",
           "Local: %lu us, Master: %lu us, "
           "Offset: %lu us",
           static_cast<unsigned long>(localTimestamp), static_cast<unsigned long>(syncMsg->timestamp),
           static_cast<unsigned long>(timeOffset));

    return nullptr;
}

// SetTime Message Handler
std::unique_ptr<Message> SetTimeMessageHandler::ProcessMessage(const Message &message, SlaveDevice *device)
{
    auto setTimeMsg = dynamic_cast<const Master2Slave::SetTimeMessage *>(&message);
    if (!setTimeMsg)
        return nullptr;

    elog_v("SetTimeMessageHandler", "Processing set time message - Timestamp: %lu us",
           static_cast<unsigned long>(setTimeMsg->timestamp));

    // 进行时间校准，计算与主机时间的偏移量
    const uint64_t localTimestamp = HptimerGetUs();
    const int64_t timeOffset = static_cast<int64_t>(setTimeMsg->timestamp) - static_cast<int64_t>(localTimestamp);

    // 存储时间偏移量用于后续采集任务
    device->m_timeOffset = timeOffset;

    elog_v("SetTimeMessageHandler",
           "Time synchronization completed - Local: %lu us, Master: %lu us, "
           "Offset: %ld us",
           static_cast<unsigned long>(localTimestamp), static_cast<unsigned long>(setTimeMsg->timestamp),
           static_cast<long>(timeOffset));

    // 创建响应消息
    auto response = std::make_unique<Slave2Master::SetTimeResponseMessage>();
    response->status = 0; // 0表示成功
    response->timestamp = setTimeMsg->timestamp;

    return std::move(response);
}

// Conduction Config Message Handler
std::unique_ptr<Message> ConductionConfigHandler::ProcessMessage(const Message &message, SlaveDevice *device)
{
    const auto configMsg = dynamic_cast<const Master2Slave::ConductionConfigMessage *>(&message);
    if (!configMsg)
        return nullptr;

    elog_v("ConductionConfigHandler", "Processing conduction configuration - Time slot: %d, Interval: %dms",
           static_cast<int>(configMsg->timeSlot), static_cast<int>(configMsg->interval));

    // 创建采集器配置（新版本只需要引脚数量和总时隙数）
    device->currentConfig =
        CollectorConfig(static_cast<uint8_t>(configMsg->conductionNum),     // 导通检测数量（本设备负责的引脚数）
                        static_cast<uint8_t>(configMsg->totalConductionNum) // 总检测数量（总时隙数）
        );

    // 配置采集器和时隙管理器
    bool collectorConfigured = device->m_continuityCollector->Configure(device->currentConfig);
    bool slotManagerConfigured = false;

    if (collectorConfigured && device->m_slotManager)
    {
        // 配置时隙管理器
        slotManagerConfigured =
            device->m_slotManager->Configure(static_cast<uint8_t>(configMsg->startConductionNum), // 起始时隙
                                             static_cast<uint8_t>(configMsg->conductionNum),      // 设备时隙数量
                                             static_cast<uint8_t>(configMsg->totalConductionNum), // 总时隙数
                                             static_cast<uint32_t>(configMsg->interval)           // 时隙间隔(ms)
            );
    }

    if (collectorConfigured && slotManagerConfigured)
    {
        device->m_isConfigured = true;
        device->m_deviceState = SlaveDeviceState::READY;

        // 重置数据发送相关状态
        device->m_hasDataToSend = false;
        device->m_isFirstCollection = true;
        device->lastCollectionData.clear();
        elog_v("ConductionConfigHandler",
               "ContinuityCollector and SlotManager configured successfully - "
               "Pins: %d, "
               "Start: %d, Total: %d, Interval: %ums",
               static_cast<int>(device->currentConfig.m_num), static_cast<int>(configMsg->startConductionNum),
               static_cast<int>(device->currentConfig.m_totalDetectionNum), static_cast<int>(configMsg->interval));
        elog_v("ConductionConfigHandler", "Configuration saved for future use. Send Sync message to start "
                                          "collection.");
    }
    else
    {
        device->m_isConfigured = false;
        device->m_deviceState = SlaveDeviceState::DEV_ERR;
        elog_e("ConductionConfigHandler", "Failed to configure ContinuityCollector or SlotManager");
    }

    auto response = std::make_unique<Slave2Master::ConductionConfigResponseMessage>();
    response->status = device->m_isConfigured ? 0 : 1; // 0=Success, 1=Error
    response->timeSlot = configMsg->timeSlot;
    response->interval = configMsg->interval;
    response->totalConductionNum = configMsg->totalConductionNum;
    response->startConductionNum = configMsg->startConductionNum;
    response->conductionNum = configMsg->conductionNum;
    return std::move(response);
}

// Resistance Config Message Handler
std::unique_ptr<Message> ResistanceConfigHandler::ProcessMessage(const Message &message, SlaveDevice *device)
{
    const auto configMsg = dynamic_cast<const Master2Slave::ResistanceConfigMessage *>(&message);
    if (!configMsg)
        return nullptr;

    elog_v("ResistanceConfigHandler", "Processing resistance configuration - Time slot: %d, Interval: %dms",
           static_cast<int>(configMsg->timeSlot), static_cast<int>(configMsg->interval));

    auto response = std::make_unique<Slave2Master::ResistanceConfigResponseMessage>();
    response->status = 0; // Success
    response->timeSlot = configMsg->timeSlot;
    response->interval = configMsg->interval;
    response->totalConductionNum = configMsg->totalNum;
    response->startConductionNum = configMsg->startNum;
    response->conductionNum = configMsg->num;
    return std::move(response);
}

// Clip Config Message Handler
std::unique_ptr<Message> ClipConfigHandler::ProcessMessage(const Message &message, SlaveDevice *device)
{
    const auto configMsg = dynamic_cast<const Master2Slave::ClipConfigMessage *>(&message);
    if (!configMsg)
        return nullptr;

    elog_v("ClipConfigHandler", "Processing clip configuration - Interval: %dms, Mode: %d",
           static_cast<int>(configMsg->interval), static_cast<int>(configMsg->mode));

    auto response = std::make_unique<Slave2Master::ClipConfigResponseMessage>();
    response->status = 0; // Success
    response->interval = configMsg->interval;
    response->mode = configMsg->mode;
    response->clipPin = configMsg->clipPin;
    return std::move(response);
}

// Ping Request Message Handler
std::unique_ptr<Message> PingRequestHandler::ProcessMessage(const Message &message, SlaveDevice *device)
{
    const auto *pingMsg = dynamic_cast<const Master2Slave::PingReqMessage *>(&message);
    if (!pingMsg)
        return nullptr;

    elog_v("PingRequestHandler", "Processing Ping request - Sequence number: %u, Timestamp: %u",
           pingMsg->sequenceNumber, pingMsg->timestamp);

    auto response = std::make_unique<Slave2Master::PingRspMessage>();
    response->sequenceNumber = pingMsg->sequenceNumber;
    response->timestamp = SlaveApp::SlaveDevice::getCurrentTimestamp();
    return std::move(response);
}

// Reset Message Handler
std::unique_ptr<Message> ResetMessageHandler::ProcessMessage(const Message &message, SlaveDevice *device)
{
    const auto *rstMsg = dynamic_cast<const Master2Slave::RstMessage *>(&message);
    if (!rstMsg)
        return nullptr;

    elog_v("ResetMessageHandler", "Processing reset message - Lock status: %d", static_cast<int>(rstMsg->lockStatus));

    // 重置设备状态，但保留配置
    device->resetDevice();

    // 创建响应消息但不立即发送，而是设置待回复标志位
    auto response = std::make_unique<Slave2Master::RstResponseMessage>();
    response->status = 0; // Success
    response->lockStatus = rstMsg->lockStatus;
    response->clipLed = rstMsg->clipLed;

    // 存储待回复的响应并设置标志位
    device->m_pendingResetResponse = std::move(response);
    device->m_hasPendingResetResponse = true;

    elog_v("ResetMessageHandler", "Reset response queued for next active slot");

    // 返回 nullptr 表示不立即回复
    return nullptr;
}

// Short ID Assignment Message Handler
std::unique_ptr<Message> ShortIdAssignHandler::ProcessMessage(const Message &message, SlaveDevice *device)
{
    const auto *assignMsg = dynamic_cast<const Master2Slave::ShortIdAssignMessage *>(&message);
    if (!assignMsg)
        return nullptr;

    elog_v("ShortIdAssignHandler", "Processing short ID assignment - Short ID: %d",
           static_cast<int>(assignMsg->shortId));

    // 直接调用SlaveDevice的setShortId方法
    device->setShortId(assignMsg->shortId);

    auto response = std::make_unique<Slave2Master::ShortIdConfirmMessage>();
    response->status = 0; // Success
    response->shortId = assignMsg->shortId;
    return std::move(response);
}

// Slave Control Message Handler
std::unique_ptr<Message> SlaveControlHandler::ProcessMessage(const Message &message, SlaveDevice *device)
{
    const auto *controlMsg = dynamic_cast<const Master2Slave::SlaveControlMessage *>(&message);
    if (!controlMsg)
        return nullptr;

    elog_i("SlaveControlHandler",
           "Processing slave control message - Mode: %d, Enable: %d, "
           "StartTime: %lu us",
           static_cast<int>(controlMsg->mode), static_cast<int>(controlMsg->enable),
           static_cast<unsigned long>(controlMsg->startTime));

    auto response = std::make_unique<Slave2Master::SlaveControlResponseMessage>();

    if (controlMsg->enable == 1)
    {
        // 启动采集
        if (!device->m_isConfigured)
        {
            elog_w("SlaveControlHandler", "Device not configured, cannot start collection");
            response->status = Slave2Master::ResponseStatusCode::FAILURE;
            return std::move(response);
        }

        // 获取当前时间

        // 检查是否已经到达启动时间
        if (const uint64_t currentTimeUs = device->GetSyncTimestampUs(); currentTimeUs >= controlMsg->startTime)
        {
            // 立即启动采集
            if (device->m_continuityCollector->StartCollection())
            {
                device->m_isCollecting = true;
                device->m_deviceState = SlaveDeviceState::RUNNING;
                elog_i("SlaveControlHandler",
                       "Data collection started immediately (start time "
                       "already reached). "
                       "Current: %lu us, Start: %lu us, Offset: %ld us",
                       static_cast<unsigned long>(currentTimeUs), static_cast<unsigned long>(controlMsg->startTime),
                       static_cast<long>(device->m_timeOffset));
                response->status = Slave2Master::ResponseStatusCode::SUCCESS;
            }
            else
            {
                elog_e("SlaveControlHandler", "Failed to start data collection");
                device->m_deviceState = SlaveDeviceState::DEV_ERR;
                response->status = Slave2Master::ResponseStatusCode::FAILURE;
            }
        }
        else
        {
            // 延迟启动：保存启动时间，等待到达时间
            device->m_scheduledStartTime = controlMsg->startTime;
            device->m_isScheduledToStart = true;
            device->m_deviceState = SlaveDeviceState::READY;

            uint64_t delayUs = controlMsg->startTime - currentTimeUs;
            elog_i("SlaveControlHandler",
                   "Data collection scheduled to start in %lu us (at %lu us). "
                   "Current: %lu us, Delay: %lu us",
                   static_cast<unsigned long>(delayUs), static_cast<unsigned long>(controlMsg->startTime),
                   static_cast<unsigned long>(currentTimeUs), static_cast<unsigned long>(delayUs));

            response->status = Slave2Master::ResponseStatusCode::SUCCESS;
        }
    }
    else
    {
        // 停止采集
        if (device->m_isCollecting)
        {
            if (device->m_continuityCollector)
            {
                device->m_continuityCollector->StopCollection();
            }
            if (device->m_slotManager)
            {
                device->m_slotManager->Stop();
            }
            device->m_isCollecting = false;
            device->m_deviceState = SlaveDeviceState::READY;

            // 重置数据发送相关状态
            device->m_hasDataToSend = false;
            device->m_isFirstCollection = true;
            device->lastCollectionData.clear();

            elog_i("SlaveControlHandler", "Data collection and slot management stopped successfully");
        }

        // 取消计划的启动
        device->m_isScheduledToStart = false;
        device->m_scheduledStartTime = 0;

        response->status = Slave2Master::ResponseStatusCode::SUCCESS;

        // 对于停止消息，立即返回响应而不是排队等待时隙
        // 因为停止后SlotManager已停止，不会再有时隙事件来发送排队的响应
        elog_v("SlaveControlHandler", "Sending stop response immediately");
        return std::move(response);
    }

    // 对于启动消息，存储待回复的响应并设置标志位，避免与数据传输冲撞
    device->m_pendingSlaveControlResponse = std::move(response);
    device->m_hasPendingSlaveControlResponse = true;

    elog_v("SlaveControlHandler", "SlaveControl response queued for next active slot");

    // 返回 nullptr 表示不立即回复
    return nullptr;
}

} // namespace SlaveApp