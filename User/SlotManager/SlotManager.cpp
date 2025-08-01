#include "SlotManager.h"
#include "elog.h"
#include "hptimer.hpp"

SlotManager::SlotManager()
    : startSlot_(0),
      deviceSlotCount_(0),
      totalSlotCount_(0),
      slotIntervalMs_(0),
      isRunning_(false),
      isConfigured_(false),
      isFirstProcess_(true),
      lastSlotTimeUs_(0) {
    elog_v("SlotManager", "SlotManager constructed");
}

bool SlotManager::configure(uint8_t startSlot, uint8_t deviceSlotCount, 
                           uint8_t totalSlotCount, uint32_t slotIntervalMs) {
    if (isRunning_) {
        elog_e("SlotManager", "Cannot configure while running");
        return false;
    }

    if (deviceSlotCount == 0 || totalSlotCount == 0 || slotIntervalMs == 0) {
        elog_e("SlotManager", "Invalid configuration parameters");
        return false;
    }

    if (startSlot >= totalSlotCount) {
        elog_e("SlotManager", "Start slot %d exceeds total slots %d", startSlot, totalSlotCount);
        return false;
    }

    if (startSlot + deviceSlotCount > totalSlotCount) {
        elog_e("SlotManager", "Device slots exceed total slots");
        return false;
    }

    startSlot_ = startSlot;
    deviceSlotCount_ = deviceSlotCount;
    totalSlotCount_ = totalSlotCount;
    slotIntervalMs_ = slotIntervalMs;
    isConfigured_ = true;

    // 初始化时隙信息
    currentSlotInfo_.totalSlots = totalSlotCount_;
    currentSlotInfo_.slotIntervalMs = slotIntervalMs_;

    elog_i("SlotManager", "Configured - Start: %d, Count: %d, Total: %d, Interval: %dms",
           startSlot_, deviceSlotCount_, totalSlotCount_, slotIntervalMs_);

    return true;
}

bool SlotManager::start() {
    if (!isConfigured_) {
        elog_e("SlotManager", "Cannot start - not configured");
        return false;
    }

    if (isRunning_) {
        elog_w("SlotManager", "Already running");
        return true;
    }

    isRunning_ = true;
    isFirstProcess_ = true;  // 标记为第一次process调用
    // 设置时间，确保第一个时隙有完整的时间间隔
    lastSlotTimeUs_ = getCurrentSyncTimeUs();
    
    // 设置初始时隙为0，但不立即触发回调
    currentSlotInfo_.currentSlot = 0;
    currentSlotInfo_.slotType = calculateSlotType(0);
    currentSlotInfo_.totalSlots = totalSlotCount_;
    currentSlotInfo_.slotIntervalMs = slotIntervalMs_;
    
    if (currentSlotInfo_.slotType == SlotType::ACTIVE) {
        // 计算激活的引脚编号（逻辑引脚）
        currentSlotInfo_.activePin = 0 - startSlot_;
        elog_v("SlotManager", "Initial ACTIVE slot 0, pin %d", currentSlotInfo_.activePin);
    } else {
        currentSlotInfo_.activePin = 0xFF; // 无效引脚
        elog_v("SlotManager", "Initial INACTIVE slot 0");
    }

    elog_i("SlotManager", "Started slot management from slot 0");
    return true;
}

void SlotManager::stop() {
    if (!isRunning_) {
        return;
    }

    isRunning_ = false;
    elog_i("SlotManager", "Stopped slot management");
}

void SlotManager::process() {
    if (!isRunning_) {
        return;
    }

    // 如果是第一次process调用，立即处理第0个时隙
    if (isFirstProcess_) {
        isFirstProcess_ = false;
        // 立即触发第0个时隙的处理
        if (slotCallback_) {
            slotCallback_(currentSlotInfo_);
        }
        elog_v("SlotManager", "First process call, handled slot 0");
        return;
    }

    uint64_t currentTimeUs = getCurrentSyncTimeUs();
    uint64_t elapsedTimeUs = currentTimeUs - lastSlotTimeUs_;
    uint64_t slotIntervalUs = slotIntervalMs_ * 1000ULL;

    // 检查是否需要切换到下一个时隙
    if (elapsedTimeUs >= slotIntervalUs) {
        uint8_t nextSlot = (currentSlotInfo_.currentSlot + 1) % totalSlotCount_;
        switchToSlot(nextSlot);
        lastSlotTimeUs_ = currentTimeUs;
    }
}

void SlotManager::setSlotCallback(SlotCallback callback) {
    slotCallback_ = callback;
}

void SlotManager::setSyncTimeCallback(SyncTimeCallback callback) {
    syncTimeCallback_ = callback;
}

uint8_t SlotManager::getCurrentActivePin() const {
    if (currentSlotInfo_.slotType == SlotType::ACTIVE) {
        return currentSlotInfo_.activePin;
    }
    return 0xFF; // 无效引脚号
}

uint64_t SlotManager::getCurrentSyncTimeUs() {
    if (syncTimeCallback_) {
        return syncTimeCallback_();
    }
    // 如果没有同步时间回调，使用本地时间
    return hal_hptimer_get_us();
}

SlotType SlotManager::calculateSlotType(uint8_t slotNumber) {
    // 检查是否在本设备的时隙范围内
    if (slotNumber >= startSlot_ && slotNumber < startSlot_ + deviceSlotCount_) {
        return SlotType::ACTIVE;
    }
    return SlotType::INACTIVE;
}

void SlotManager::switchToSlot(uint8_t newSlot) {
    currentSlotInfo_.currentSlot = newSlot;
    currentSlotInfo_.slotType = calculateSlotType(newSlot);
    
    if (currentSlotInfo_.slotType == SlotType::ACTIVE) {
        // 计算激活的引脚编号（逻辑引脚）
        currentSlotInfo_.activePin = newSlot - startSlot_;
        elog_d("SlotManager", "Switched to ACTIVE slot %d, pin %d", 
               newSlot, currentSlotInfo_.activePin, (unsigned long)(getCurrentSyncTimeUs() / 1000));
    } else {
        currentSlotInfo_.activePin = 0xFF; // 无效引脚
        elog_d("SlotManager", "Switched to INACTIVE slot %d", 
               newSlot, (unsigned long)(getCurrentSyncTimeUs() / 1000));
    }

    // 触发回调
    if (slotCallback_) {
        slotCallback_(currentSlotInfo_);
    }
}