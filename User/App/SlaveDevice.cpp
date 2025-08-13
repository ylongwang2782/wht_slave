#include "SlaveDevice.h"

#include <cstdint>
#include <cstdio>

#include "Master2SlaveMessageHandlers.h"
#include "adc.h"
#include "cmsis_os2.h"
#include "elog.h"
#include "hal_uid.hpp"
#include "hptimer.hpp"
#include "main.h"
#include "uwb_task.h"

using namespace WhtsProtocol;

// 发送数据入队超时
#define MsgProc_TX_QUEUE_TIMEOUT 1000
// 发送超时
#define MsgProc_TX_TIMEOUT 1000

namespace SlaveApp {

SlaveDevice::SlaveDevice()
    : deviceId(DeviceUID::get()),    // 自动读取设备UID
      shortId(0),                    // 初始短ID为0，表示未分配
      isJoined(false),               // 初始未入网
      isConfigured(false),
      deviceState(SlaveDeviceState::IDLE),
      timeOffset(0),                // 初始时间偏移量为0
      isCollecting(false),          // 初始未在采集
      scheduledStartTime(0),        // 初始计划启动时间为0
      isScheduledToStart(false),    // 初始未计划启动
      hasDataToSend(false),         // 初始无数据待发送
      isFirstCollection(true),      // 初始为第一次采集
      deviceStatus({}) {            // 初始化设备状态

    // Initialize continuity collector
    continuityCollector = ContinuityCollectorFactory::create();

    // Initialize slot manager
    slotManager = std::make_unique<SlotManager>();

    // 设置时隙管理器的同步时间回调
    if (slotManager) {
        slotManager->setSyncTimeCallback(
            [this]() { return this->getSyncTimestampUs(); });

        // 设置时隙切换回调
        slotManager->setSlotCallback([this](const SlotInfo& slotInfo) {
            this->onSlotChanged(slotInfo);
        });
    }

    // set MTU = 1016
    processor.setMTU(800);

    // Initialize message handlers
    initializeMessageHandlers();

    dataCollectionTask = std::make_unique<DataCollectionTask>(*this);
    announceTask = std::make_unique<AnnounceTask>(*this);
    joinTask = std::make_unique<JoinTask>(*this);
    slaveDataProcT = std::make_unique<SlaveDataProcT>(*this);
    accessoryTask = std::make_unique<AccessoryTask>(*this);
}

void SlaveDevice::initializeMessageHandlers() {
    messageHandlers_[static_cast<uint8_t>(
        WhtsProtocol::Master2SlaveMessageId::SYNC_MSG)] =
        &SyncMessageHandler::getInstance();
    messageHandlers_[static_cast<uint8_t>(
        WhtsProtocol::Master2SlaveMessageId::SET_TIME_MSG)] =
        &SetTimeMessageHandler::getInstance();
    messageHandlers_[static_cast<uint8_t>(
        WhtsProtocol::Master2SlaveMessageId::CONDUCTION_CFG_MSG)] =
        &ConductionConfigHandler::getInstance();
    messageHandlers_[static_cast<uint8_t>(
        WhtsProtocol::Master2SlaveMessageId::RESISTANCE_CFG_MSG)] =
        &ResistanceConfigHandler::getInstance();
    messageHandlers_[static_cast<uint8_t>(
        WhtsProtocol::Master2SlaveMessageId::CLIP_CFG_MSG)] =
        &ClipConfigHandler::getInstance();
    messageHandlers_[static_cast<uint8_t>(
        WhtsProtocol::Master2SlaveMessageId::PING_REQ_MSG)] =
        &PingRequestHandler::getInstance();
    messageHandlers_[static_cast<uint8_t>(
        WhtsProtocol::Master2SlaveMessageId::RST_MSG)] =
        &ResetMessageHandler::getInstance();
    messageHandlers_[static_cast<uint8_t>(
        WhtsProtocol::Master2SlaveMessageId::SHORT_ID_ASSIGN_MSG)] =
        &ShortIdAssignHandler::getInstance();
    messageHandlers_[static_cast<uint8_t>(
        WhtsProtocol::Master2SlaveMessageId::SLAVE_CONTROL_MSG)] =
        &SlaveControlHandler::getInstance();
}

std::unique_ptr<Message> SlaveDevice::processMaster2SlaveMessage(
    const Message& message) {
    elog_v("SlaveDevice", "Processing Master2Slave message: %s",
           message.getMessageTypeName());

    uint8_t messageId = message.getMessageId();
    elog_v("SlaveDevice", "Processing Master2Slave message, ID: 0x%02X",
           static_cast<int>(messageId));

    IMaster2SlaveMessageHandler* handler = messageHandlers_[messageId];
    if (handler) {
        // Process message using the appropriate handler
        auto response = handler->processMessage(message, this);

        if (response) {
            elog_v("SlaveDevice",
                   "Response generated for Master2Slave message");
        } else {
            elog_v("SlaveDevice",
                   "No response needed for this Master2Slave message");
        }

        return response;
    } else {
        elog_w("SlaveDevice", "Unknown Master2Slave message type: 0x%02X",
               static_cast<int>(messageId));
        return nullptr;
    }
}

uint32_t SlaveDevice::getCurrentTimestamp() {
    uint32_t tick = hal_hptimer_get_us();
    return tick;
}

uint64_t SlaveDevice::getSyncTimestampUs() {
    // 获取当前本地时间（微秒）
    uint64_t localTimeUs = hal_hptimer_get_us();

    // 应用时间偏移量得到同步时间（微秒）
    uint64_t syncTimeUs = localTimeUs + timeOffset;

    // 转换为毫秒
    return syncTimeUs;
}

uint32_t SlaveDevice::getSyncTimestampMs() {
    // 获取当前本地时间（微秒）
    uint64_t localTimeUs = hal_hptimer_get_us();

    // 应用时间偏移量得到同步时间（微秒）
    uint64_t syncTimeUs = localTimeUs + timeOffset;

    // 转换为毫秒
    return static_cast<uint32_t>(syncTimeUs / 1000);
}

void SlaveDevice::resetDevice() {
    // 保留配置，但重置状态
    deviceState = SlaveDeviceState::READY;

    // 停止采集相关状态
    if (isCollecting && continuityCollector) {
        continuityCollector->stopCollection();
        isCollecting = false;
    }

    // 停止时隙管理器
    if (slotManager) {
        slotManager->stop();
    }

    // 重置数据发送相关状态
    hasDataToSend = false;
    isFirstCollection = true;
    lastCollectionData.clear();

    // 重置启动调度相关状态
    isScheduledToStart = false;
    scheduledStartTime = 0;

    elog_v("SlaveDevice",
           "Device reset to READY state, configuration preserved");
}

void SlaveDevice::onSlotChanged(const SlotInfo& slotInfo) {
    // 只在采集状态下处理时隙事件
    if (!isCollecting || !continuityCollector || !slotManager) {
        return;
    }

    // 检查是否是本设备的第一个激活时隙，且有数据待发送
    if (slotInfo.slotType == SlotType::ACTIVE &&
        slotInfo.activePin == 0 &&    // 第一个激活引脚
        hasDataToSend && !isFirstCollection) {
        elog_v(TAG,
               "Reached own first active slot, sending cached data to backend");

        // 发送缓存的数据
        if (dataCollectionTask) {
            dataCollectionTask->sendDataToBackend();
        }
        hasDataToSend = false;
    }

    // 通知采集器处理当前时隙
    continuityCollector->processSlot(slotInfo.currentSlot, slotInfo.activePin,
                                     slotInfo.slotType == SlotType::ACTIVE);

    // 检查采集是否完成
    if (continuityCollector->isCollectionComplete()) {
        elog_v(TAG, "Data collection cycle completed");

        // 如果不是第一次采集，保存数据准备在下次自己的时隙发送
        if (!isFirstCollection) {
            auto dataVector = continuityCollector->getDataVector();
            lastCollectionData = dataVector;
            hasDataToSend = true;
            elog_v(TAG, "Saved %d bytes of data for next transmission",
                   dataVector.size());
        } else {
            // 第一次采集完成，标记为非第一次
            isFirstCollection = false;
            elog_v(TAG, "First collection completed, no data to send yet");
        }

        // 清空数据矩阵并准备下一轮采集
        continuityCollector->clearData();

        // 重新启动采集器
        if (continuityCollector->startCollection()) {
            elog_v(TAG, "Starting new data collection cycle");
        } else {
            elog_e(TAG, "Failed to start new data collection cycle");
            isCollecting = false;
            deviceState = SlaveDeviceState::DEV_ERR;
            if (slotManager) {
                slotManager->stop();
            }
        }
    }
}

void SlaveDevice::setShortId(uint8_t id) {
    shortId = id;
    isJoined = true;
    elog_d(TAG, "Short ID assigned: %d, device joined successfully", shortId);
}

void SlaveDevice::processFrame(Frame& frame) {
    elog_v("SlaveDevice",
           "Processing frame - PacketId: 0x%02X, payload size: %d",
           static_cast<int>(frame.packetId), frame.payload.size());

    if (frame.packetId == static_cast<uint8_t>(PacketId::MASTER_TO_SLAVE)) {
        uint32_t targetSlaveId;
        std::unique_ptr<Message> masterMessage;

        if (processor.parseMaster2SlavePacket(frame.payload, targetSlaveId,
                                              masterMessage)) {
            // Check if this message is for us (or broadcast)
            if (targetSlaveId == deviceId || targetSlaveId == BROADCAST_ID) {
                elog_d("SlaveDevice",
                       "Received Master2Slave message for device 0x%08X, "
                       "Message: %s (ID: 0x%02X)",
                       targetSlaveId, masterMessage->getMessageTypeName(),
                       static_cast<int>(masterMessage->getMessageId()));

                // Process message and create response
                auto response = processMaster2SlaveMessage(*masterMessage);

                if (response) {
                    elog_v("SlaveDevice", "Generated response message");

                    std::vector<std::vector<uint8_t>> responseData;

                    elog_v("SlaveDevice", "Packing Slave2Master message: %s",
                           response->getMessageTypeName());
                    responseData =
                        processor.packSlave2MasterMessage(deviceId, *response);

                    elog_v("SlaveDevice", "Sending response:");

                    // Send all fragments to master
                    for (auto& fragment : responseData) {
                        if (!send(fragment)) {
                            elog_e("SlaveDevice",
                                   "Failed to send response fragment");
                        }
                    }
                }
            } else {
                elog_v("SlaveDevice",
                       "Message not for this device (target: 0x%08X, our ID: "
                       "0x%08X)",
                       targetSlaveId, deviceId);
            }
        } else {
            elog_e("SlaveDevice", "Failed to parse Master2Slave packet");
        }
    } else {
        // elog_w("SlaveDevice", "Unsupported packet type for Slave: 0x%02X",
        //        static_cast<int>(frame.packetId));
    }
}

void SlaveDevice::run() {
    if (dataCollectionTask) {
        dataCollectionTask->give();
        elog_d(TAG, "DataCollectionTask initialized and started");
    }

    if (announceTask) {
        announceTask->give();
        elog_d(TAG, "AnnounceTask initialized and started");
    }

    if (joinTask) {
        joinTask->give();
        elog_d(TAG, "JoinTask initialized and started");
    }

    if (slaveDataProcT) {
        slaveDataProcT->give();
        elog_d(TAG, "SlaveDataProcT initialized and started");
    }

    if (accessoryTask) {
        accessoryTask->give();
        elog_d(TAG, "AccessoryTask initialized and started");
    }

    while (1) {
        // elog_d(TAG, "hptimer ms: %d", hal_hptimer_get_ms());

        HAL_GPIO_TogglePin(RUN_LED_GPIO_Port, RUN_LED_Pin);
        HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
        TaskBase::delay(500);
    }
}

// AnnounceTask 实现
SlaveDevice::AnnounceTask::AnnounceTask(SlaveDevice& parent)
    : TaskClassS("AnnounceTask", TaskPrio_Mid), parent(parent) {}

void SlaveDevice::AnnounceTask::task() {
    uint8_t announceCount = 0;

    // 等待数据传输任务启动
    TaskBase::delay(1000);

    while (announceCount < MAX_ANNOUNCE_COUNT) {
        if (sendAnnounceMessage()) {
            elog_d(TAG, "Announce message sent successfully (%d/%d)",
                   announceCount + 1, MAX_ANNOUNCE_COUNT);
        } else {
            elog_e(TAG, "Failed to send announce message (%d/%d)",
                   announceCount + 1, MAX_ANNOUNCE_COUNT);
        }

        announceCount++;

        if (announceCount < MAX_ANNOUNCE_COUNT) {
            TaskBase::delay(ANNOUNCE_INTERVAL_MS);
        }
    }

    elog_d(TAG, "Announce task completed, sent %d messages", announceCount);

    // 任务完成后挂起自己
    vTaskSuspend(nullptr);
}

bool SlaveDevice::AnnounceTask::sendAnnounceMessage() {
    // 创建 AnnounceMessage
    WhtsProtocol::Slave2Master::AnnounceMessage announceMsg;
    announceMsg.deviceId = parent.deviceId;
    announceMsg.versionMajor = FIRMWARE_VERSION_MAJOR;
    announceMsg.versionMinor = FIRMWARE_VERSION_MINOR;
    announceMsg.versionPatch = FIRMWARE_VERSION_PATCH;

    elog_d(TAG, "Creating announce message: ID=0x%08X, Version=%d.%d.%d",
           announceMsg.deviceId, announceMsg.versionMajor,
           announceMsg.versionMinor, announceMsg.versionPatch);

    // 使用协议处理器打包消息
    DeviceStatus deviceStatus = {};
    auto packedData =
        parent.processor.packSlave2MasterMessage(parent.deviceId, announceMsg);

    // 发送所有数据包片段
    bool success = true;
    for (auto& fragment : packedData) {
        if (!parent.send(fragment)) {
            elog_e(TAG, "Failed to send announce message fragment");
            success = false;
            break;
        }
    }

    return success;
}

// JoinTask 实现
SlaveDevice::JoinTask::JoinTask(SlaveDevice& parent)
    : TaskClassS("JoinTask", TaskPrio_Mid), parent(parent) {}

void SlaveDevice::JoinTask::task() {
    elog_d(TAG, "Join task started, waiting for network join...");

    // 等待入网完成
    if (waitForJoin()) {
        elog_d(TAG, "Device successfully joined network with short ID: %d",
               parent.shortId);
    } else {
        elog_w(TAG, "Join timeout, device will continue without short ID");
    }

    // 任务完成后挂起自己
    vTaskSuspend(nullptr);
}

bool SlaveDevice::JoinTask::waitForJoin() {
    uint32_t startTime = hal_hptimer_get_ms();
    uint32_t timeoutMs = JOIN_TIMEOUT_MS;

    while ((hal_hptimer_get_ms() - startTime) < timeoutMs) {
        if (parent.isJoined) {
            return true;
        }
        TaskBase::delay(JOIN_CHECK_INTERVAL_MS);
    }

    return false;
}

void SlaveDevice::JoinTask::handleShortIdAssign(uint8_t assignedShortId) {
    elog_d(TAG, "Received short ID assignment: %d", assignedShortId);

    // 设置短ID
    parent.setShortId(assignedShortId);

    // 发送确认消息
    if (sendShortIdConfirm(assignedShortId, 0)) {    // 0表示成功
        elog_d(TAG, "Short ID confirm sent successfully");
    } else {
        elog_e(TAG, "Failed to send short ID confirm");
    }
}

bool SlaveDevice::JoinTask::sendShortIdConfirm(uint8_t shortId,
                                               uint8_t status) {
    // 创建 ShortIdConfirmMessage
    WhtsProtocol::Slave2Master::ShortIdConfirmMessage confirmMsg;
    confirmMsg.shortId = shortId;
    confirmMsg.status = status;

    elog_d(TAG, "Sending short ID confirm: shortId=%d, status=%d", shortId,
           status);

    // 使用协议处理器打包消息
    auto packedData =
        parent.processor.packSlave2MasterMessage(parent.deviceId, confirmMsg);

    // 发送所有数据包片段
    bool success = true;
    for (auto& fragment : packedData) {
        if (!parent.send(fragment)) {
            elog_e(TAG, "Failed to send short ID confirm fragment");
            success = false;
            break;
        }
    }

    return success;
}

// DataCollectionTask 实现
SlaveDevice::DataCollectionTask::DataCollectionTask(SlaveDevice& parent)
    : TaskClassS("DataCollectionTask", TaskPrio_Mid), parent(parent) {}

void SlaveDevice::DataCollectionTask::task() {
    elog_d(TAG, "Data collection task started");

    for (;;) {
        // 处理数据采集状态
        processDataCollection();

        // 处理时隙管理器状态
        if (parent.slotManager && parent.isCollecting) {
            parent.slotManager->process();
        }

        // 减少轮询间隔以提高时隙切换精度
        TaskBase::delay(1);
    }
}

void SlaveDevice::DataCollectionTask::processDataCollection() {
    // 检查是否有计划启动的采集
    if (parent.isScheduledToStart && !parent.isCollecting) {
        uint64_t currentTimeUs = parent.getSyncTimestampUs();

        // 检查是否到达启动时间
        if (currentTimeUs >= parent.scheduledStartTime) {
            elog_i(TAG,
                   "Scheduled start time reached, starting data collection. "
                   "Current: %lu us, Scheduled: %lu us",
                   (unsigned long)currentTimeUs,
                   (unsigned long)parent.scheduledStartTime);

            // 启动采集和时隙管理器
            if (parent.continuityCollector && parent.slotManager &&
                parent.continuityCollector->startCollection() &&
                parent.slotManager->start()) {
                parent.isCollecting = true;
                parent.deviceState = SlaveDeviceState::RUNNING;
                parent.isScheduledToStart = false;
                parent.scheduledStartTime = 0;
                parent.isFirstCollection = true;    // 重置为第一次采集
                elog_i(TAG,
                       "Data collection and slot management started "
                       "successfully from scheduled start");
            } else {
                elog_e(TAG,
                       "Failed to start scheduled data collection or slot "
                       "management");
                parent.deviceState = SlaveDeviceState::DEV_ERR;
                parent.isScheduledToStart = false;
                parent.scheduledStartTime = 0;
            }
        }
    }

    // processDataCollection方法现在主要负责启动调度
    // 实际的采集和数据处理由时隙管理器通过onSlotChanged回调处理
}

void SlaveDevice::DataCollectionTask::sendDataToBackend() {
    if (!parent.isConfigured) {
        elog_w(TAG, "Device not configured");
        return;
    }

    // 检查是否有缓存的数据可发送
    if (!parent.hasDataToSend || parent.lastCollectionData.empty()) {
        elog_w(TAG, "No cached data available to send to backend");
        return;
    }

    // 根据当前配置创建相应的数据消息
    // 这里假设是导通检测模式，实际应该根据配置的模式来决定
    auto dataMsg = std::make_unique<Slave2Backend::ConductionDataMessage>();

    // 使用缓存的数据
    dataMsg->conductionData = parent.lastCollectionData;
    dataMsg->conductionLength = dataMsg->conductionData.size();

    if (dataMsg->conductionLength > 0) {
        elog_v(TAG, "Sending %d bytes of cached conduction data to backend",
               dataMsg->conductionLength);

        // 使用协议处理器打包消息为Slave2Backend格式
        auto packedData = parent.processor.packSlave2BackendMessage(
            parent.deviceId, parent.deviceStatus, *dataMsg);

        // 发送所有数据包片段
        bool success = true;
        for (size_t i = 0; i < packedData.size(); ++i) {
            if (!parent.send(packedData[i])) {
                elog_e(TAG, "Failed to send data fragment %d/%d to backend",
                       i + 1, packedData.size());
                success = false;
                break;
            }
        }

        if (success) {
            elog_v(TAG,
                   "Cached data successfully sent to backend (%d fragments)",
                   packedData.size());
            // 发送成功后清空缓存数据
            parent.lastCollectionData.clear();
        } else {
            elog_e(TAG, "Failed to send complete cached data to backend");
        }
    } else {
        elog_w(TAG, "No cached data available to send to backend");
    }
}

bool SlaveDevice::send(std::vector<uint8_t>& frame) {
    // UWB_SendData to send
    UWB_SendData(frame.data(), frame.size(), 0);

    return true;
}

// SlaveDataProcT 实现
SlaveDevice::SlaveDataProcT::SlaveDataProcT(SlaveDevice& parent)
    : TaskClassS("SlaveDataProcT", TaskPrio_Mid), parent(parent) {}

void SlaveDevice::SlaveDataProcT::task() {
    elog_i(TAG, "SlaveDataProcT started");
    uwb_rx_msg_t msg;
    for (;;) {
        if (UWB_ReceiveData(&msg, 0) == 0) {
            elog_v(TAG, "SlaveDataProcT recvData size: %d", msg.data_len);
            // copy msg.data to recvData
            recvData.assign(msg.data, msg.data + msg.data_len);

            if (!recvData.empty()) {
                // process recvData
                elog_d(TAG, "recvData size: %d", recvData.size());
                parent.processor.processReceivedData(recvData);

                // process complete frame
                Frame receivedFrame;
                while (parent.processor.getNextCompleteFrame(receivedFrame)) {
                    parent.processFrame(receivedFrame);
                }
                recvData.clear();
            }
        }
        TaskBase::delay(1);
    }
}

// AccessoryTask 实现
SlaveDevice::AccessoryTask::AccessoryTask(SlaveDevice& parent)
    : TaskClassS("AccessoryTask", TaskPrio_Mid),
      parent(parent),
      key1("Key1", KEY1_GPIO_Port, KEY1_Pin),
      unlockBtn("unlockBtn", UNLOCK_BTN_GPIO_Port, UNLOCK_BTN_Pin),
      auxBtn1("auxBtn1", AUX_BTN1_GPIO_Port, AUX_BTN1_Pin),
      auxBtn2("auxBtn2", AUX_BTN2_GPIO_Port, AUX_BTN2_Pin),
      pSensor("pSensor", P_SENSOR_GPIO_Port, P_SENSOR_Pin),
      clrSensor("clrSensor", CLR_SENSOR_GPIO_Port, CLR_SENSOR_Pin),
      valve1("Valve1", VALVE1_GPIO_Port, VALVE1_Pin),
      dipInfo{{
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
      lockController("LockController", key1, unlockBtn, valve1)

{}

void SlaveDevice::AccessoryTask::task() {
    elog_d(TAG, "Accessory hardware initialized");

    for (;;) {
        // 自动处理逻辑
        lockController.update();

        // 更新设备状态
        updateDeviceStatus();

        osDelay(UPDATE_INTERVAL_MS);
    }
}

void SlaveDevice::AccessoryTask::updateDeviceStatus() {
    parent.deviceStatus.pressureSensor = pSensor.isTrigger();
    parent.deviceStatus.clrSensor = clrSensor.isTrigger();
    parent.deviceStatus.accessory1 = auxBtn1.isPressed();
    parent.deviceStatus.accessory2 = auxBtn2.isPressed();
    parent.deviceStatus.electromagneticLock1 =
        (lockController.getState() == LockState::Locked);
    parent.deviceStatus.electromagneticLock2 = false;
    parent.deviceStatus.electromagnetUnlockButton = unlockBtn.isPressed();
    parent.deviceStatus.sleeveLimit = false;

    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    adc_value = HAL_ADC_GetValue(&hadc1);
    battery_voltage = (adc_value / 4095.0f) * 3.15f * 2.0f;

    // 判断逻辑（带滞回）
    if (!parent.deviceStatus.batteryLowAlarm &&
        battery_voltage < BATTERY_VOLTAGE_THRESHOLD) {
        parent.deviceStatus.batteryLowAlarm = true;
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
    } else if (parent.deviceStatus.batteryLowAlarm &&
               battery_voltage > BATTERY_VOLTAGE_THRESHOLD +
                                     BATTERY_VOLTAGE_THRESHOLD_OFFSET) {
        parent.deviceStatus.batteryLowAlarm = false;
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
    }
}

}    // namespace SlaveApp