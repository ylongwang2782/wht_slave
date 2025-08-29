#include "master_slave_message_handlers.h"

#include "slave_device.h"
#include "elog.h"
#include "hptimer.hpp"

using namespace WhtsProtocol;

namespace SlaveApp {

// Sync Message Handler
std::unique_ptr<Message> SyncMessageHandler::processMessage(
    const Message &message, SlaveDevice *device) {
    auto syncMsg = dynamic_cast<const Master2Slave::SyncMessage *>(&message);
    if (!syncMsg) return nullptr;

    elog_v("SyncMessageHandler", "Processing sync message - Timestamp: %lu us",
           (unsigned long)syncMsg->timestamp);

    // 进行时间校准，计算与主机时间的偏移量
    uint64_t localTimestamp = hal_hptimer_get_us();
    int64_t timeOffset = static_cast<int64_t>(syncMsg->timestamp) -
                         static_cast<int64_t>(localTimestamp);

    // 存储时间偏移量用于后续采集任务
    device->timeOffset = timeOffset;

    elog_v("SyncMessageHandler",
           "Time synchronization completed - Local: %lu us, Master: %lu us, "
           "Offset: %ld us",
           (unsigned long)localTimestamp, (unsigned long)syncMsg->timestamp,
           (long)timeOffset);

    return nullptr;
}

// SetTime Message Handler
std::unique_ptr<Message> SetTimeMessageHandler::processMessage(
    const Message &message, SlaveDevice *device) {
    auto setTimeMsg =
        dynamic_cast<const Master2Slave::SetTimeMessage *>(&message);
    if (!setTimeMsg) return nullptr;

    elog_v("SetTimeMessageHandler",
           "Processing set time message - Timestamp: %lu us",
           (unsigned long)setTimeMsg->timestamp);

    // 进行时间校准，计算与主机时间的偏移量
    uint64_t localTimestamp = hal_hptimer_get_us();
    int64_t timeOffset = static_cast<int64_t>(setTimeMsg->timestamp) -
                         static_cast<int64_t>(localTimestamp);

    // 存储时间偏移量用于后续采集任务
    device->timeOffset = timeOffset;

    elog_v("SetTimeMessageHandler",
           "Time synchronization completed - Local: %lu us, Master: %lu us, "
           "Offset: %ld us",
           (unsigned long)localTimestamp, (unsigned long)setTimeMsg->timestamp,
           (long)timeOffset);

    // 创建响应消息
    auto response = std::make_unique<Slave2Master::SetTimeResponseMessage>();
    response->status = 0;    // 0表示成功
    response->timestamp = setTimeMsg->timestamp;

    return std::move(response);
}

// Conduction Config Message Handler
std::unique_ptr<Message> ConductionConfigHandler::processMessage(
    const Message &message, SlaveDevice *device) {
    auto configMsg =
        dynamic_cast<const Master2Slave::ConductionConfigMessage *>(&message);
    if (!configMsg) return nullptr;

    elog_v(
        "ConductionConfigHandler",
        "Processing conduction configuration - Time slot: %d, Interval: %dms",
        static_cast<int>(configMsg->timeSlot),
        static_cast<int>(configMsg->interval));

    // 创建采集器配置（新版本只需要引脚数量和总时隙数）
    device->currentConfig = CollectorConfig(
        static_cast<uint8_t>(configMsg->conductionNum),         // 导通检测数量（本设备负责的引脚数）
        static_cast<uint8_t>(configMsg->totalConductionNum)     // 总检测数量（总时隙数）
    );

    // 配置采集器和时隙管理器
    bool collectorConfigured = device->continuityCollector->configure(device->currentConfig);
    bool slotManagerConfigured = false;
    
    if (collectorConfigured && device->slotManager) {
        // 配置时隙管理器
        slotManagerConfigured = device->slotManager->configure(
            static_cast<uint8_t>(configMsg->startConductionNum),    // 起始时隙
            static_cast<uint8_t>(configMsg->conductionNum),         // 设备时隙数量
            static_cast<uint8_t>(configMsg->totalConductionNum),    // 总时隙数
            static_cast<uint32_t>(configMsg->interval)              // 时隙间隔(ms)
        );
    }

    if (collectorConfigured && slotManagerConfigured) {
        device->isConfigured = true;
        device->deviceState = SlaveDeviceState::READY;
        
        // 重置数据发送相关状态
        device->hasDataToSend = false;
        device->isFirstCollection = true;
        device->lastCollectionData.clear();
        elog_v("ConductionConfigHandler",
               "ContinuityCollector and SlotManager configured successfully - Pins: %d, "
               "Start: %d, Total: %d, Interval: %ums",
               static_cast<int>(device->currentConfig.num),
               static_cast<int>(configMsg->startConductionNum),
               static_cast<int>(device->currentConfig.totalDetectionNum),
               static_cast<int>(configMsg->interval));
        elog_v("ConductionConfigHandler",
               "Configuration saved for future use. Send Sync message to start "
               "collection.");
    } else {
        device->isConfigured = false;
        device->deviceState = SlaveDeviceState::DEV_ERR;
        elog_e("ConductionConfigHandler",
               "Failed to configure ContinuityCollector or SlotManager");
    }

    auto response =
        std::make_unique<Slave2Master::ConductionConfigResponseMessage>();
    response->status = device->isConfigured ? 0 : 1;    // 0=Success, 1=Error
    response->timeSlot = configMsg->timeSlot;
    response->interval = configMsg->interval;
    response->totalConductionNum = configMsg->totalConductionNum;
    response->startConductionNum = configMsg->startConductionNum;
    response->conductionNum = configMsg->conductionNum;
    return std::move(response);
}

// Resistance Config Message Handler
std::unique_ptr<Message> ResistanceConfigHandler::processMessage(
    const Message &message, SlaveDevice *device) {
    auto configMsg =
        dynamic_cast<const Master2Slave::ResistanceConfigMessage *>(&message);
    if (!configMsg) return nullptr;

    elog_v(
        "ResistanceConfigHandler",
        "Processing resistance configuration - Time slot: %d, Interval: %dms",
        static_cast<int>(configMsg->timeSlot),
        static_cast<int>(configMsg->interval));

    auto response =
        std::make_unique<Slave2Master::ResistanceConfigResponseMessage>();
    response->status = 0;    // Success
    response->timeSlot = configMsg->timeSlot;
    response->interval = configMsg->interval;
    response->totalConductionNum = configMsg->totalNum;
    response->startConductionNum = configMsg->startNum;
    response->conductionNum = configMsg->num;
    return std::move(response);
}

// Clip Config Message Handler
std::unique_ptr<Message> ClipConfigHandler::processMessage(
    const Message &message, SlaveDevice *device) {
    auto configMsg =
        dynamic_cast<const Master2Slave::ClipConfigMessage *>(&message);
    if (!configMsg) return nullptr;

    elog_v("ClipConfigHandler",
           "Processing clip configuration - Interval: %dms, Mode: %d",
           static_cast<int>(configMsg->interval),
           static_cast<int>(configMsg->mode));

    auto response = std::make_unique<Slave2Master::ClipConfigResponseMessage>();
    response->status = 0;    // Success
    response->interval = configMsg->interval;
    response->mode = configMsg->mode;
    response->clipPin = configMsg->clipPin;
    return std::move(response);
}

// Note: ReadConductionDataHandler, ReadResistanceDataHandler, and
// ReadClipDataHandler have been removed as they conflict with the new
// push-based data collection architecture. Data is now automatically pushed by
// slaves via DataCollectionTask instead of being pulled by master.

// Ping Request Message Handler
std::unique_ptr<Message> PingRequestHandler::processMessage(
    const Message &message, SlaveDevice *device) {
    const auto *pingMsg =
        dynamic_cast<const Master2Slave::PingReqMessage *>(&message);
    if (!pingMsg) return nullptr;

    elog_v("PingRequestHandler",
           "Processing Ping request - Sequence number: %u, Timestamp: %u",
           pingMsg->sequenceNumber, pingMsg->timestamp);

    auto response = std::make_unique<Slave2Master::PingRspMessage>();
    response->sequenceNumber = pingMsg->sequenceNumber;
    response->timestamp = device->getCurrentTimestamp();
    return std::move(response);
}

// Reset Message Handler
std::unique_ptr<Message> ResetMessageHandler::processMessage(
    const Message &message, SlaveDevice *device) {
    const auto *rstMsg =
        dynamic_cast<const Master2Slave::RstMessage *>(&message);
    if (!rstMsg) return nullptr;

    elog_v("ResetMessageHandler", "Processing reset message - Lock status: %d",
           static_cast<int>(rstMsg->lockStatus));

    // 重置设备状态，但保留配置
    device->resetDevice();

    // 创建响应消息但不立即发送，而是设置待回复标志位
    auto response = std::make_unique<Slave2Master::RstResponseMessage>();
    response->status = 0;    // Success
    response->lockStatus = rstMsg->lockStatus;
    response->clipLed = rstMsg->clipLed;
    
    // 存储待回复的响应并设置标志位
    device->pendingResetResponse = std::move(response);
    device->hasPendingResetResponse = true;
    
    elog_v("ResetMessageHandler", "Reset response queued for next active slot");
    
    // 返回 nullptr 表示不立即回复
    return nullptr;
}

// Short ID Assignment Message Handler
std::unique_ptr<Message> ShortIdAssignHandler::processMessage(
    const Message &message, SlaveDevice *device) {
    const auto *assignMsg =
        dynamic_cast<const Master2Slave::ShortIdAssignMessage *>(&message);
    if (!assignMsg) return nullptr;

    elog_v("ShortIdAssignHandler",
           "Processing short ID assignment - Short ID: %d",
           static_cast<int>(assignMsg->shortId));

    // 直接调用SlaveDevice的setShortId方法
    device->setShortId(assignMsg->shortId);

    auto response = std::make_unique<Slave2Master::ShortIdConfirmMessage>();
    response->status = 0;    // Success
    response->shortId = assignMsg->shortId;
    return std::move(response);
}

// Slave Control Message Handler
std::unique_ptr<Message> SlaveControlHandler::processMessage(
    const Message &message, SlaveDevice *device) {
    const auto *controlMsg =
        dynamic_cast<const Master2Slave::SlaveControlMessage *>(&message);
    if (!controlMsg) return nullptr;

    elog_i("SlaveControlHandler",
           "Processing slave control message - Mode: %d, Enable: %d, "
           "StartTime: %lu us",
           static_cast<int>(controlMsg->mode),
           static_cast<int>(controlMsg->enable),
           (unsigned long)controlMsg->startTime);

    auto response =
        std::make_unique<Slave2Master::SlaveControlResponseMessage>();

    if (controlMsg->enable == 1) {
        // 启动采集
        if (!device->isConfigured) {
            elog_w("SlaveControlHandler",
                   "Device not configured, cannot start collection");
            response->status = Slave2Master::ResponseStatusCode::FAILURE;
            return std::move(response);
        }

        // 获取当前时间
        uint64_t currentTimeUs = device->getSyncTimestampUs();

        // 检查是否已经到达启动时间
        if (currentTimeUs >= controlMsg->startTime) {
            // 立即启动采集
            if (device->continuityCollector->startCollection()) {
                device->isCollecting = true;
                device->deviceState = SlaveDeviceState::RUNNING;
                elog_i("SlaveControlHandler",
                       "Data collection started immediately (start time "
                       "already reached). "
                       "Current: %lu us, Start: %lu us, Offset: %ld us",
                       (unsigned long)currentTimeUs,
                       (unsigned long)controlMsg->startTime,
                       (long)device->timeOffset);
                response->status = Slave2Master::ResponseStatusCode::SUCCESS;
            } else {
                elog_e("SlaveControlHandler",
                       "Failed to start data collection");
                device->deviceState = SlaveDeviceState::DEV_ERR;
                response->status = Slave2Master::ResponseStatusCode::FAILURE;
            }
        } else {
            // 延迟启动：保存启动时间，等待到达时间
            device->scheduledStartTime = controlMsg->startTime;
            device->isScheduledToStart = true;
            device->deviceState = SlaveDeviceState::READY;

            uint64_t delayUs = controlMsg->startTime - currentTimeUs;
            elog_i("SlaveControlHandler",
                   "Data collection scheduled to start in %lu us (at %lu us). "
                   "Current: %lu us, Delay: %lu us",
                   (unsigned long)delayUs, (unsigned long)controlMsg->startTime,
                   (unsigned long)currentTimeUs, (unsigned long)delayUs);

            response->status = Slave2Master::ResponseStatusCode::SUCCESS;
        }
    } else {
        // 停止采集
        if (device->isCollecting) {
            if (device->continuityCollector) {
                device->continuityCollector->stopCollection();
            }
            if (device->slotManager) {
                device->slotManager->stop();
            }
            device->isCollecting = false;
            device->deviceState = SlaveDeviceState::READY;
            
            // 重置数据发送相关状态
            device->hasDataToSend = false;
            device->isFirstCollection = true;
            device->lastCollectionData.clear();
            
            elog_i("SlaveControlHandler",
                   "Data collection and slot management stopped successfully");
        }

        // 取消计划的启动
        device->isScheduledToStart = false;
        device->scheduledStartTime = 0;

        response->status = Slave2Master::ResponseStatusCode::SUCCESS;
        
        // 对于停止消息，立即返回响应而不是排队等待时隙
        // 因为停止后SlotManager已停止，不会再有时隙事件来发送排队的响应
        elog_v("SlaveControlHandler", "Sending stop response immediately");
        return std::move(response);
    }

    // 对于启动消息，存储待回复的响应并设置标志位，避免与数据传输冲撞
    device->pendingSlaveControlResponse = std::move(response);
    device->hasPendingSlaveControlResponse = true;
    
    elog_v("SlaveControlHandler", "SlaveControl response queued for next active slot");
    
    // 返回 nullptr 表示不立即回复
    return nullptr;
}

}    // namespace SlaveApp