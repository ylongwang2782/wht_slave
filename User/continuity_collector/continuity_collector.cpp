#include "continuity_collector.h"
#include <algorithm>
#include <map>
#include <memory>

#include "FreeRTOS.h"
#include "elog.h"
#include "task.h"

// 端口时钟使能函数
static void enableGpioPortClock(GPIO_TypeDef *port)
{
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

// 将逻辑引脚号转换为GPIO端口和引脚
GpioPin CollectorConfig::GetGpioPin(uint8_t logicalPin)
{
    if (logicalPin < 64)
    {
        // 直接使用映射表中的GPIO信息
        return GpioPin(HARDWARE_PIN_MAP[logicalPin].m_port, HARDWARE_PIN_MAP[logicalPin].m_pin);
    }

    // 默认返回PA0
    elog_e("CollectorConfig", "Invalid logical pin: %d", logicalPin);
    return GpioPin(GPIOA, GPIO_PIN_0);
}

ContinuityCollector::ContinuityCollector() : m_status(CollectionStatus::IDLE), m_currentCycle(0), m_lastActivePin(-1)
{
    elog_v(TAG, "Constructor: config_.num: %d", m_config.m_num);
}

ContinuityCollector::~ContinuityCollector()
{
    StopCollection();
    DeinitializeGpioPins();
}

bool ContinuityCollector::Configure(const CollectorConfig &config)
{
    // if (status_ == CollectionStatus::RUNNING) {
    //     return false;    // 不能在运行时重新配置
    // }

    if (config.m_num == 0 || config.m_num > MAX_GPIO_PINS)
    {
        return false;
    }

    if (config.m_totalDetectionNum == 0 || config.m_totalDetectionNum > 65535)
    {
        return false;
    }

    m_config = config;

    // 优化内存分配策略 - 预分配所有内存
    {
        m_dataMatrix.clear();
        m_dataMatrix.reserve(m_config.m_totalDetectionNum);

        // 预分配所有行的内存，避免动态增长
        for (uint16_t i = 0; i < m_config.m_totalDetectionNum; ++i)
        {
            m_dataMatrix.emplace_back(m_config.m_num, ContinuityState::DISCONNECTED);
        }

        // 监控内存使用情况
        size_t totalElements = m_config.m_totalDetectionNum * m_config.m_num;
        size_t totalBytes = totalElements * sizeof(ContinuityState);
        elog_v(TAG, "Memory allocated: %d rows x %d cols = %d elements (%d bytes)", m_config.m_totalDetectionNum,
               m_config.m_num, totalElements, totalBytes);
    }

    m_currentCycle = 0;

    // 初始化GPIO引脚
    InitializeGpioPins();

    m_status = CollectionStatus::IDLE;

    return true;
}

bool ContinuityCollector::StartCollection()
{
    if (m_status == CollectionStatus::RUNNING)
    {
        return false;
    }

    if (m_config.m_num == 0)
    {
        return false;
    }

    // 停止之前的采集
    StopCollection();

    // 重置状态
    m_currentCycle = 0;
    m_status = CollectionStatus::RUNNING;
    m_lastActivePin = -1; // 重置上一个激活的引脚

    elog_v(TAG, "startCollection completed, status: RUNNING");
    return true;
}

void ContinuityCollector::StopCollection()
{
    if (m_status == CollectionStatus::RUNNING)
    {
        m_status = CollectionStatus::IDLE;
        // 复位所有引脚
        if (m_lastActivePin >= 0)
        {
            GpioPin gpioPin = m_config.GetGpioPin(m_lastActivePin);
            HalGpioInit(gpioPin, GPIO_MODE_INPUT, GPIO_PULLDOWN);
            m_lastActivePin = -1;
        }
    }
}

// uint32_t ContinuityCollector::getCurrentTimeMs() {
//     return hal_hptimer_get_ms();
// }

// uint64_t ContinuityCollector::getCurrentTimeUs() {
//     return hal_hptimer_get_us();
// }

void ContinuityCollector::DelayMs(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

// 处理时隙事件（由外部时隙管理器调用）
void ContinuityCollector::ProcessSlot(uint16_t slotNumber, uint8_t activePin, bool isActive)
{
    // 只在运行状态下处理
    if (m_status != CollectionStatus::RUNNING)
    {
        return;
    }

    // 检查是否已完成所有周期
    if (m_currentCycle >= m_config.m_totalDetectionNum)
    {
        m_status = CollectionStatus::COMPLETED;
        return;
    }

    // 配置当前时隙的引脚状态
    ConfigurePinsForSlot(activePin, isActive);

    DelayMs(3);

    // 使用静态缓冲区避免频繁的内存分配
    static std::vector<ContinuityState> slotData;
    slotData.clear();
    slotData.reserve(m_config.m_num);

    // 读取当前时隙的所有引脚状态
    for (uint8_t pin = 0; pin < m_config.m_num; pin++)
    {
        ContinuityState state = ReadPinContinuity(pin);
        slotData.push_back(state);
    }

    // 保存数据到矩阵 - 直接复制，避免移动操作
    if (m_currentCycle < m_dataMatrix.size())
    {
        m_dataMatrix[m_currentCycle] = slotData;
    }

    // 减少日志输出频率，只在每10个周期输出一次
    if (m_currentCycle % 10 == 0)
    {
        // elog_d(TAG, "Processed slot %d (cycle %d), active: %s, pin: %d", slotNumber,
        //        currentCycle_, isActive ? "true" : "false", activePin);
    }

    // 更新周期计数
    m_currentCycle++;

    // 检查是否完成
    if (m_currentCycle >= m_config.m_totalDetectionNum)
    {
        m_status = CollectionStatus::COMPLETED;
        // // 复位最后一个激活的引脚
        // if (lastActivePin_ >= 0 && lastActivePin_ < config_.num) {
        //     GpioPin gpioPin = config_.getGpioPin(lastActivePin_);
        //     halGpioInit(gpioPin, GPIO_MODE_INPUT, GPIO_PULLDOWN);
        //     lastActivePin_ = -1;
        // }
        // elog_v("ContinuityCollector", "Data collection completed after %d
        // slots", currentCycle_);
    }

    // 触发进度回调
    if (m_progressCallback)
    {
        m_progressCallback(m_currentCycle, m_config.m_totalDetectionNum);
    }
}

CollectionStatus ContinuityCollector::GetStatus() const
{
    return m_status;
}

uint16_t ContinuityCollector::GetCurrentCycle() const
{
    return m_currentCycle;
}

uint16_t ContinuityCollector::GetTotalCycles() const
{
    return m_config.m_totalDetectionNum;
}

float ContinuityCollector::GetProgress() const
{
    uint16_t current = GetCurrentCycle();
    uint16_t total = GetTotalCycles();
    if (total == 0)
        return 0.0f;
    return static_cast<float>(current * 100) / total;
}

bool ContinuityCollector::HasNewData() const
{
    return m_status == CollectionStatus::COMPLETED;
}

bool ContinuityCollector::IsCollectionComplete() const
{
    return m_status == CollectionStatus::COMPLETED;
}

ContinuityMatrix ContinuityCollector::GetDataMatrix() const
{
    return m_dataMatrix;
}

std::vector<uint8_t> ContinuityCollector::GetDataVector() const
{
    std::vector<uint8_t> compressedData;

    size_t totalBits = m_dataMatrix.size() * m_config.m_num;
    size_t totalBytes = (totalBits + 7) / 8; // 向上取整
    compressedData.reserve(totalBytes);

    uint8_t currentByte = 0;
    uint8_t bitPosition = 7; // 从高位开始（大端）

    for (const auto &row : m_dataMatrix)
    {
        for (size_t pin = 0; pin < m_config.m_num && pin < row.size(); pin++)
        {
            uint8_t bitValue = (row[pin] == ContinuityState::CONNECTED) ? 1 : 0;

            // 设置对应的高位
            currentByte |= (bitValue << bitPosition);

            if (bitPosition == 0)
            {
                compressedData.push_back(currentByte);
                currentByte = 0;
                bitPosition = 7;
            }
            else
            {
                bitPosition--;
            }
        }
    }

    // 处理不足8位的最后一个字节
    if (bitPosition != 7)
    {
        compressedData.push_back(currentByte);
    }

    return compressedData;
}

std::vector<ContinuityState> ContinuityCollector::GetCycleData(uint16_t cycle) const
{
    if (cycle < m_dataMatrix.size())
    {
        return m_dataMatrix[cycle];
    }
    return {};
}

std::vector<ContinuityState> ContinuityCollector::GetPinData(uint8_t pin) const
{
    std::vector<ContinuityState> result;

    if (pin < m_config.m_num)
    {
        result.reserve(m_dataMatrix.size());
        for (const auto &row : m_dataMatrix)
        {
            if (pin < row.size())
            {
                result.push_back(row[pin]);
            }
        }
    }

    return result;
}

void ContinuityCollector::ClearData()
{
    for (auto &row : m_dataMatrix)
    {
        std::fill(row.begin(), row.end(), ContinuityState::DISCONNECTED);
    }
    m_currentCycle = 0;
}

void ContinuityCollector::SetProgressCallback(ProgressCallback callback)
{
    m_progressCallback = callback;
}

ContinuityCollector::Statistics ContinuityCollector::CalculateStatistics() const
{
    Statistics stats = {};

    uint32_t totalConnections = 0;
    uint32_t totalReadings = 0;
    std::map<uint8_t, uint32_t> pinActivity;

    // 统计数据
    for (const auto &row : m_dataMatrix)
    {
        for (uint8_t pin = 0; pin < row.size(); pin++)
        {
            totalReadings++;
            if (row[pin] == ContinuityState::CONNECTED)
            {
                totalConnections++;
                pinActivity[pin]++;
            }
        }
    }

    stats.totalConnections = totalConnections;
    stats.totalDisconnections = totalReadings - totalConnections;
    stats.connectionRate = totalReadings > 0 ? static_cast<double>(totalConnections) / totalReadings * 100.0 : 0.0;

    // 找出最活跃的引脚
    std::vector<std::pair<uint8_t, uint32_t>> sortedPins;
    for (const auto &pair : pinActivity)
    {
        sortedPins.push_back(pair);
    }

    std::sort(sortedPins.begin(), sortedPins.end(), [](const auto &a, const auto &b) { return a.second > b.second; });

    for (size_t i = 0; i < 5 && i < sortedPins.size(); i++)
    {
        stats.mostActivePins[i] = sortedPins[i].first;
    }

    return stats;
}

void ContinuityCollector::InitializeGpioPins()
{
    elog_v(TAG, "initializeGpioPins");

    elog_v(TAG, "initializeGpioPins: config_.num: %d", m_config.m_num);

    // 初始化所有需要的GPIO引脚为输入模式
    for (uint8_t logicalPin = 0; logicalPin < m_config.m_num; logicalPin++)
    {
        elog_v(TAG, "logicalPin: %d", logicalPin);
        GpioPin gpioPin = m_config.GetGpioPin(logicalPin);
        elog_v(TAG, "GPIO port: %p, pin: 0x%04x", gpioPin.m_port, gpioPin.m_pin);
        HalGpioInit(gpioPin, GPIO_MODE_INPUT, GPIO_PULLDOWN);
        elog_v(TAG, "GPIO initialized");
    }
}

void ContinuityCollector::DeinitializeGpioPins()
{
    // 反初始化所有GPIO引脚
    for (uint8_t logicalPin = 0; logicalPin < m_config.m_num; logicalPin++)
    {
        GpioPin gpioPin = m_config.GetGpioPin(logicalPin);
        HalGpioDeinit(gpioPin);
    }
}

ContinuityState ContinuityCollector::ReadPinContinuity(uint8_t logicalPin)
{
    if (logicalPin >= m_config.m_num)
    {
        return ContinuityState::DISCONNECTED;
    }

    GpioPin gpioPin = m_config.GetGpioPin(logicalPin);
    GPIO_PinState pinState = HalGpioRead(gpioPin);

    // 高电平表示导通，低电平表示断开
    return (pinState == GPIO_PIN_SET) ? ContinuityState::CONNECTED : ContinuityState::DISCONNECTED;
}

void ContinuityCollector::ConfigurePinsForSlot(uint8_t activePin, bool isActive)
{
    // // 1. 立即复位上一个激活的引脚（如果有）
    // if (lastActivePin_ >= 0 && lastActivePin_ < config_.num) {
    //     GpioPin gpioPin = config_.getGpioPin(lastActivePin_);
    //     halGpioInit(gpioPin, GPIO_MODE_INPUT, GPIO_PULLDOWN);
    //     elog_v(TAG, "Reset previous active pin: logical=%d", lastActivePin_);
    // }

    // 复位所有已配置引脚
    for (uint8_t pin = 0; pin < m_config.m_num; pin++)
    {
        GpioPin gpioPin = m_config.GetGpioPin(pin);
        HalGpioInit(gpioPin, GPIO_MODE_INPUT, GPIO_PULLDOWN);
    }

    // 2. 如果当前时隙是激活时隙，配置对应引脚为输出高电平
    if (isActive && activePin < m_config.m_num)
    {
        GpioPin activeGpioPin = m_config.GetGpioPin(activePin);
        HalGpioInit(activeGpioPin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_PIN_SET);
        HalGpioWrite(activeGpioPin, GPIO_PIN_SET);

        elog_v(TAG, "Activated pin logical=%d", activePin);
        m_lastActivePin = activePin;
    }
    else
    {
        m_lastActivePin = -1;
    }
}

// HAL库GPIO辅助函数实现
void ContinuityCollector::HalGpioInit(const GpioPin &gpioPin, uint32_t mode, uint32_t pull, GPIO_PinState initialState)
{
    if (!gpioPin.m_port)
        return;

    // 使能对应端口的时钟
    enableGpioPortClock(gpioPin.m_port);

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = gpioPin.m_pin;
    GPIO_InitStruct.Mode = mode;
    GPIO_InitStruct.Pull = pull;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    // 如果是输出模式且指定了初始状态，先设置引脚状态
    if ((mode == GPIO_MODE_OUTPUT_PP || mode == GPIO_MODE_OUTPUT_OD) && initialState != GPIO_PIN_RESET)
    {
        HAL_GPIO_WritePin(gpioPin.m_port, gpioPin.m_pin, initialState);
    }

    HAL_GPIO_Init(gpioPin.m_port, &GPIO_InitStruct);
}

void ContinuityCollector::HalGpioDeinit(const GpioPin &gpioPin)
{
    if (!gpioPin.m_port)
        return;
    HAL_GPIO_DeInit(gpioPin.m_port, gpioPin.m_pin);
}

GPIO_PinState ContinuityCollector::HalGpioRead(const GpioPin &gpioPin)
{
    if (!gpioPin.m_port)
        return GPIO_PIN_RESET;
    return HAL_GPIO_ReadPin(gpioPin.m_port, gpioPin.m_pin);
}

void ContinuityCollector::HalGpioWrite(const GpioPin &gpioPin, GPIO_PinState state)
{
    if (!gpioPin.m_port)
        return;
    HAL_GPIO_WritePin(gpioPin.m_port, gpioPin.m_pin, state);
}

// 工厂类实现
std::unique_ptr<ContinuityCollector> ContinuityCollectorFactory::Create()
{
    return std::make_unique<ContinuityCollector>();
}