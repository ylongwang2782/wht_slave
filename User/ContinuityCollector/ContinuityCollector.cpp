#include "ContinuityCollector.h"

#include <stdint.h>

#include <algorithm>
#include <map>
#include <memory>

#include "FreeRTOS.h"
#include "elog.h"
#include "hptimer.hpp"
#include "task.h"

// GPIO端口映射表
static GPIO_TypeDef *GPIO_PORTS[] = {GPIOA, GPIOB, GPIOC, GPIOD,
                                     GPIOE, GPIOF, GPIOG};

// 端口时钟使能函数
static void enableGpioPortClock(GPIO_TypeDef *port) {
    if (port == GPIOA)
        __HAL_RCC_GPIOA_CLK_ENABLE();
    else if (port == GPIOB)
        __HAL_RCC_GPIOB_CLK_ENABLE();
    else if (port == GPIOC)
        __HAL_RCC_GPIOC_CLK_ENABLE();
    else if (port == GPIOD)
        __HAL_RCC_GPIOD_CLK_ENABLE();
    else if (port == GPIOE)
        __HAL_RCC_GPIOE_CLK_ENABLE();
    else if (port == GPIOF)
        __HAL_RCC_GPIOF_CLK_ENABLE();
    else if (port == GPIOG)
        __HAL_RCC_GPIOG_CLK_ENABLE();
}

// 将物理引脚号转换为GPIO端口和引脚
GpioPin CollectorConfig::getGpioPin(uint8_t logicalPin) const {
    uint8_t physicalPin = getPhysicalPin(logicalPin);

    // 计算端口索引和引脚号
    uint8_t portIndex =
        physicalPin / 16;    // PA=0, PB=1, PC=2, PD=3, PE=4, PF=5, PG=6
    uint8_t pinNumber = physicalPin % 16;    // 引脚号 0-15

    if (portIndex < 7) {
        GPIO_TypeDef *port = GPIO_PORTS[portIndex];
        uint16_t pin = (1 << pinNumber);    // 转换为GPIO_PIN_x格式
        return GpioPin(port, pin);
    }

    // 默认返回PA0
    elog_e("CollectorConfig", "Invalid physical pin: %d", physicalPin);
    return GpioPin(GPIOA, GPIO_PIN_0);
}

ContinuityCollector::ContinuityCollector()
    : status_(CollectionStatus::IDLE),
      currentCycle_(0),
      lastProcessTime_(0),
      lastActivePin_(-1) {
    elog_v("ContinuityCollector", "Constructor: config_.num: %d", config_.num);
}

ContinuityCollector::~ContinuityCollector() {
    stopCollection();
    deinitializeGpioPins();
}

bool ContinuityCollector::configure(const CollectorConfig &config) {
    // if (status_ == CollectionStatus::RUNNING) {
    //     return false;    // 不能在运行时重新配置
    // }

    if (config.num == 0 || config.num > MAX_GPIO_PINS || config.interval == 0) {
        return false;
    }

    if (config.totalDetectionNum == 0 ||
        config.totalDetectionNum > MAX_GPIO_PINS) {
        return false;
    }

    if (config.startDetectionNum >= config.totalDetectionNum) {
        return false;
    }

    config_ = config;

    // 重新初始化数据矩阵 - 基于总检测数量
    {
        dataMatrix_.clear();
        dataMatrix_.resize(config_.totalDetectionNum,
                           std::vector<ContinuityState>(
                               config_.num, ContinuityState::DISCONNECTED));
    }

    currentCycle_ = 0;

    // 初始化GPIO引脚
    initializeGpioPins();

    status_ = CollectionStatus::IDLE;

    return true;
}

bool ContinuityCollector::startCollection() {
    if (status_ == CollectionStatus::RUNNING) {
        return false;
    }

    if (config_.num == 0) {
        return false;
    }

    // 停止之前的采集
    stopCollection();

    // 重置状态
    currentCycle_ = 0;
    status_ = CollectionStatus::RUNNING;
    lastProcessTime_ = getSyncTimeUs();    // 使用同步时间
    lastActivePin_ = -1;                   // 重置上一个激活的引脚

    elog_v("ContinuityCollector", "startCollection completed, status: RUNNING");
    return true;
}

void ContinuityCollector::stopCollection() {
    if (status_ == CollectionStatus::RUNNING) {
        status_ = CollectionStatus::IDLE;
        // 复位所有引脚
        if (lastActivePin_ >= 0) {
            GpioPin gpioPin = config_.getGpioPin(lastActivePin_);
            halGpioInit(gpioPin, GPIO_MODE_INPUT, GPIO_PULLDOWN);
            lastActivePin_ = -1;
        }
    }
}

uint32_t ContinuityCollector::getCurrentTimeMs() {
    return hal_hptimer_get_ms();
}

uint64_t ContinuityCollector::getCurrentTimeUs() {
    return hal_hptimer_get_us();
}

uint32_t ContinuityCollector::getSyncTimeMs() {
    // 如果设置了同步时间回调，使用同步时间，否则使用本地时间
    if (syncTimeCallback_) {
        return syncTimeCallback_();
    }
    return getCurrentTimeMs();
}

uint64_t ContinuityCollector::getSyncTimeUs() {
    // 如果设置了同步时间回调，使用同步时间，否则使用本地时间
    if (syncTimeCallback_) {
        return syncTimeCallback_();
    }
    return getCurrentTimeUs();
}

void ContinuityCollector::delayMs(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

// 状态机处理方法
void ContinuityCollector::processCollection() {
    // 只处理RUNNING状态
    if (status_ != CollectionStatus::RUNNING) {
        return;
    }

    // 检查是否已完成所有周期
    if (currentCycle_ >= config_.totalDetectionNum) {
        status_ = CollectionStatus::COMPLETED;
        return;
    }

    // 检查是否到了下一个采集周期的时间
    uint64_t currentTime = getSyncTimeUs();    // 使用同步时间
    uint64_t elapsedTime = currentTime - lastProcessTime_;

    if (elapsedTime >= config_.interval || currentCycle_ == 0) {
        // 立即复位上一个激活的引脚，然后配置新的引脚
        configurePinsForCycle(currentCycle_);

        // 延迟5ms，确保电路稳定并抵消时差
        delayMs(1);

        // 读取当前周期的所有引脚状态
        std::vector<ContinuityState> cycleData;
        cycleData.reserve(config_.num);
        // elog_d("ContinuityCollector", "Cycle %d: Reading %d pins after 5ms
        // delay",
        //        currentCycle_, config_.num);

        for (uint8_t pin = 0; pin < config_.num; pin++) {
            ContinuityState state = readPinContinuity(pin);
            cycleData.push_back(state);
            // elog_d("ContinuityCollector", "Cycle %d, Pin %d (physical %d):
            // %s",
            //        currentCycle_, pin, config_.getPhysicalPin(pin),
            //        (state == ContinuityState::CONNECTED) ? "CONNECTED" :
            //        "DISCONNECTED");
        }

        // 保存数据到矩阵
        {
            if (currentCycle_ < dataMatrix_.size()) {
                dataMatrix_[currentCycle_] = std::move(cycleData);
            }
        }


        // 更新时间和周期
        lastProcessTime_ = currentTime;
        currentCycle_++;

        // 检查是否完成
        if (currentCycle_ >= config_.totalDetectionNum) {
            status_ = CollectionStatus::COMPLETED;
            // 复位最后一个激活的引脚
            if (lastActivePin_ >= 0) {
                GpioPin gpioPin = config_.getGpioPin(lastActivePin_);
                halGpioInit(gpioPin, GPIO_MODE_INPUT, GPIO_PULLDOWN);
                lastActivePin_ = -1;
            }
        }
    }
}

CollectionStatus ContinuityCollector::getStatus() const { return status_; }

uint8_t ContinuityCollector::getCurrentCycle() const { return currentCycle_; }

uint8_t ContinuityCollector::getTotalCycles() const {
    return config_.totalDetectionNum;
}

float ContinuityCollector::getProgress() const {
    uint8_t current = getCurrentCycle();
    uint8_t total = getTotalCycles();
    if (total == 0) return 0.0f;
    return static_cast<float>(current * 100) / total;
}

bool ContinuityCollector::hasNewData() const {
    return status_ == CollectionStatus::COMPLETED;
}

bool ContinuityCollector::isCollectionComplete() const {
    return status_ == CollectionStatus::COMPLETED;
}

ContinuityMatrix ContinuityCollector::getDataMatrix() const {
    return dataMatrix_;
}

std::vector<uint8_t> ContinuityCollector::getDataVector() const {
    std::vector<uint8_t> compressedData;

    size_t totalBits = dataMatrix_.size() * config_.num;
    size_t totalBytes = (totalBits + 7) / 8;    // 向上取整
    compressedData.reserve(totalBytes);

    uint8_t currentByte = 0;
    uint8_t bitPosition = 7;    // 从高位开始（大端）

    for (const auto &row : dataMatrix_) {
        for (size_t pin = 0; pin < config_.num && pin < row.size(); pin++) {
            uint8_t bitValue = (row[pin] == ContinuityState::CONNECTED) ? 1 : 0;

            // 设置对应的高位
            currentByte |= (bitValue << bitPosition);

            if (bitPosition == 0) {
                compressedData.push_back(currentByte);
                currentByte = 0;
                bitPosition = 7;
            } else {
                bitPosition--;
            }
        }
    }

    // 处理不足8位的最后一个字节
    if (bitPosition != 7) {
        compressedData.push_back(currentByte);
    }

    return compressedData;
}

std::vector<ContinuityState> ContinuityCollector::getCycleData(
    uint8_t cycle) const {
    if (cycle < dataMatrix_.size()) {
        return dataMatrix_[cycle];
    }
    return {};
}

std::vector<ContinuityState> ContinuityCollector::getPinData(
    uint8_t pin) const {
    std::vector<ContinuityState> result;

    if (pin < config_.num) {
        result.reserve(dataMatrix_.size());
        for (const auto &row : dataMatrix_) {
            if (pin < row.size()) {
                result.push_back(row[pin]);
            }
        }
    }

    return result;
}

void ContinuityCollector::clearData() {
    for (auto &row : dataMatrix_) {
        std::fill(row.begin(), row.end(), ContinuityState::DISCONNECTED);
    }
    currentCycle_ = 0;
}

void ContinuityCollector::setProgressCallback(ProgressCallback callback) {
    progressCallback_ = callback;
}

void ContinuityCollector::setSyncTimeCallback(SyncTimeCallback callback) {
    syncTimeCallback_ = callback;
}

ContinuityCollector::Statistics ContinuityCollector::calculateStatistics()
    const {
    Statistics stats = {};

    uint32_t totalConnections = 0;
    uint32_t totalReadings = 0;
    std::map<uint8_t, uint32_t> pinActivity;

    // 统计数据
    for (const auto &row : dataMatrix_) {
        for (uint8_t pin = 0; pin < row.size(); pin++) {
            totalReadings++;
            if (row[pin] == ContinuityState::CONNECTED) {
                totalConnections++;
                pinActivity[pin]++;
            }
        }
    }

    stats.totalConnections = totalConnections;
    stats.totalDisconnections = totalReadings - totalConnections;
    stats.connectionRate =
        totalReadings > 0
            ? static_cast<double>(totalConnections) / totalReadings * 100.0
            : 0.0;

    // 找出最活跃的引脚
    std::vector<std::pair<uint8_t, uint32_t>> sortedPins;
    for (const auto &pair : pinActivity) {
        sortedPins.push_back(pair);
    }

    std::sort(sortedPins.begin(), sortedPins.end(),
              [](const auto &a, const auto &b) { return a.second > b.second; });

    for (size_t i = 0; i < 5 && i < sortedPins.size(); i++) {
        stats.mostActivePins[i] = sortedPins[i].first;
    }

    return stats;
}

void ContinuityCollector::initializeGpioPins() {
    elog_v("ContinuityCollector", "initializeGpioPins");

    elog_v("ContinuityCollector", "initializeGpioPins: config_.num: %d",
           config_.num);

    // 初始化所有需要的GPIO引脚为输入模式
    for (uint8_t logicalPin = 0; logicalPin < config_.num; logicalPin++) {
        elog_v("ContinuityCollector", "logicalPin: %d", logicalPin);
        GpioPin gpioPin = config_.getGpioPin(logicalPin);
        elog_v("ContinuityCollector", "GPIO port: %p, pin: 0x%04x",
               gpioPin.port, gpioPin.pin);
        halGpioInit(gpioPin, GPIO_MODE_INPUT, GPIO_PULLDOWN);
        elog_v("ContinuityCollector", "GPIO initialized");
    }
}

void ContinuityCollector::deinitializeGpioPins() {
    // 反初始化所有GPIO引脚
    for (uint8_t logicalPin = 0; logicalPin < config_.num; logicalPin++) {
        GpioPin gpioPin = config_.getGpioPin(logicalPin);
        halGpioDeinit(gpioPin);
    }
}

ContinuityState ContinuityCollector::readPinContinuity(uint8_t logicalPin) {
    if (logicalPin >= config_.num) {
        return ContinuityState::DISCONNECTED;
    }

    GpioPin gpioPin = config_.getGpioPin(logicalPin);
    GPIO_PinState pinState = halGpioRead(gpioPin);

    // 高电平表示导通，低电平表示断开
    return (pinState == GPIO_PIN_SET) ? ContinuityState::CONNECTED
                                      : ContinuityState::DISCONNECTED;
}

void ContinuityCollector::configurePinsForCycle(uint8_t currentCycle) {
    // 1. 立即复位上一个激活的引脚（如果有）
    if (lastActivePin_ >= 0) {
        GpioPin gpioPin = config_.getGpioPin(lastActivePin_);
        halGpioInit(gpioPin, GPIO_MODE_INPUT, GPIO_PULLDOWN);
        elog_v("ContinuityCollector", "Reset previous active pin: logical=%d",
               lastActivePin_);
    }

    // 2. 确定当前周期应该激活的引脚
    int8_t currentActivePin = -1;
    if (currentCycle >= config_.startDetectionNum &&
        currentCycle < config_.startDetectionNum + config_.num) {
        currentActivePin = currentCycle - config_.startDetectionNum;
    }

    // 3. 先配置激活引脚为输出高电平（如果有），再配置其他引脚为输入
    if (currentActivePin >= 0) {
        GpioPin activeGpioPin = config_.getGpioPin(currentActivePin);
        halGpioInit(activeGpioPin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,
                    GPIO_PIN_SET);
        halGpioWrite(activeGpioPin, GPIO_PIN_SET);

        elog_v("ContinuityCollector", "Cycle %d: Activated pin logical=%d",
               currentCycle, currentActivePin);
    }

    // 4. 配置其他引脚为输入下拉模式
    for (uint8_t logicalPin = 0; logicalPin < config_.num; logicalPin++) {
        if (logicalPin != currentActivePin) {    // 跳过已激活的引脚
            GpioPin gpioPin = config_.getGpioPin(logicalPin);
            halGpioInit(gpioPin, GPIO_MODE_INPUT, GPIO_PULLDOWN);
        }
    }

    // 5. 更新上一个激活的引脚记录
    lastActivePin_ = currentActivePin;
}

// HAL库GPIO辅助函数实现
void ContinuityCollector::halGpioInit(const GpioPin &gpioPin, uint32_t mode,
                                      uint32_t pull,
                                      GPIO_PinState initialState) {
    if (!gpioPin.port) return;

    // 使能对应端口的时钟
    enableGpioPortClock(gpioPin.port);

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = gpioPin.pin;
    GPIO_InitStruct.Mode = mode;
    GPIO_InitStruct.Pull = pull;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    // 如果是输出模式且指定了初始状态，先设置引脚状态
    if ((mode == GPIO_MODE_OUTPUT_PP || mode == GPIO_MODE_OUTPUT_OD) &&
        initialState != GPIO_PIN_RESET) {
        HAL_GPIO_WritePin(gpioPin.port, gpioPin.pin, initialState);
    }

    HAL_GPIO_Init(gpioPin.port, &GPIO_InitStruct);
}

void ContinuityCollector::halGpioDeinit(const GpioPin &gpioPin) {
    if (!gpioPin.port) return;
    HAL_GPIO_DeInit(gpioPin.port, gpioPin.pin);
}

GPIO_PinState ContinuityCollector::halGpioRead(const GpioPin &gpioPin) {
    if (!gpioPin.port) return GPIO_PIN_RESET;
    return HAL_GPIO_ReadPin(gpioPin.port, gpioPin.pin);
}

void ContinuityCollector::halGpioWrite(const GpioPin &gpioPin,
                                       GPIO_PinState state) {
    if (!gpioPin.port) return;
    HAL_GPIO_WritePin(gpioPin.port, gpioPin.pin, state);
}

// 工厂类实现
std::unique_ptr<ContinuityCollector> ContinuityCollectorFactory::create() {
    return std::make_unique<ContinuityCollector>();
}