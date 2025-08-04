#ifndef CONTINUITY_COLLECTOR_H
#define CONTINUITY_COLLECTOR_H

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "elog.h"
#include "main.h"

// 导通状态枚举
enum class ContinuityState : uint8_t {
    DISCONNECTED = 0,    // 断开
    CONNECTED = 1        // 导通
};

// GPIO端口和引脚映射结构
struct GpioPin {
    GPIO_TypeDef* port;
    uint16_t pin;

    GpioPin(GPIO_TypeDef* p = nullptr, uint16_t pin_num = 0)
        : port(p), pin(pin_num) {}
};

// 硬件定义的引脚映射表 (64个引脚)
static constexpr uint8_t HARDWARE_PIN_MAP[64] = {
    // PA3 - PA12, PA15
    3, 4, 5, 6, 7,           // PA3-PA7
    8, 9, 10, 11, 12, 15,    // PA8-PA12, PA15

    // PB0-1, PB10-PB15
    16 + 0, 16 + 1,                                          // PB0-PB1
    16 + 10, 16 + 11, 16 + 12, 16 + 13, 16 + 14, 16 + 15,    // PB10-PB15

    // PC4-PC11
    32 + 4, 32 + 5, 32 + 6, 32 + 7, 32 + 8, 32 + 9, 32 + 10,
    32 + 11,    // PC4-PC11

    // PD0-1, PD3-PD4, PD8-PD15
    48 + 0, 48 + 1, 48 + 3, 48 + 4,        // PD0-1, PD3-PD4
    48 + 8, 48 + 9, 48 + 10, 48 + 11,      // PD8-PD11
    48 + 12, 48 + 13, 48 + 14, 48 + 15,    // PD12-PD15

    // PE7-PE15
    64 + 7, 64 + 8, 64 + 9, 64 + 10, 64 + 11, 64 + 12, 64 + 13, 64 + 14,
    64 + 15,    // PE7-PE15

    // PF11-PF15
    80 + 11, 80 + 12, 80 + 13, 80 + 14, 80 + 15,    // PF11-PF15

    // PG0-PG8
    96 + 0, 96 + 1, 96 + 2, 96 + 3, 96 + 4, 96 + 5, 96 + 6, 96 + 7,
    96 + 8    // PG0-PG8
};

// 导通数据采集配置
struct CollectorConfig {
    uint8_t num;                  // 导通检测数量 (本设备负责的引脚数量)
    uint16_t totalDetectionNum;    // 总检测数量 (整个采集周期的时隙数量)

    CollectorConfig(uint8_t n = 2, uint16_t totalDetNum = 4)
        : num(n),
          totalDetectionNum(totalDetNum) {
        if (num > 64) num = 64;
        if (totalDetectionNum == 0 || totalDetectionNum > 65535)
            totalDetectionNum = 65535;

        elog_v("CollectorConfig", "Constructor: num=%d, totalDetectionNum=%d", num, totalDetectionNum);
    }

    // 获取逻辑引脚对应的物理引脚
    uint8_t getPhysicalPin(uint8_t logicalPin) const {
        elog_v("CollectorConfig", "getPhysicalPin called, logicalPin: %d",
               logicalPin);

        if (logicalPin < 64) {
            uint8_t physicalPin = HARDWARE_PIN_MAP[logicalPin];
            elog_v("CollectorConfig", "returning physicalPin: %d", physicalPin);
            return physicalPin;
        }

        elog_v("CollectorConfig", "logicalPin out of range, returning: %d",
               logicalPin);
        return logicalPin;    // 如果超出范围，返回逻辑引脚号
    }

    // 将物理引脚号转换为GPIO端口和引脚
    GpioPin getGpioPin(uint8_t logicalPin) const;
};

// 导通数据矩阵类型
using ContinuityMatrix = std::vector<std::vector<ContinuityState>>;

// 采集状态枚举
enum class CollectionStatus : uint8_t {
    IDLE = 0,         // 空闲状态
    RUNNING = 1,      // 正在采集
    COMPLETED = 2,    // 采集完成
    ERROR = 3         // 错误状态
};

// 采集进度回调函数类型
using ProgressCallback =
    std::function<void(uint16_t cycle, uint16_t totalCycles)>;

// 时间同步回调函数类型 - 用于获取同步时间
using SyncTimeCallback = std::function<uint64_t()>;

// 导通数据采集器类
class ContinuityCollector {
   private:
   // define TAG
    static constexpr const char* TAG = "ConCollector";
    static constexpr uint8_t MAX_GPIO_PINS = 64;

    CollectorConfig config_;         // 采集配置
    ContinuityMatrix dataMatrix_;    // 数据矩阵

    CollectionStatus status_;              // 采集状态
    uint16_t currentCycle_;                 // 当前周期
    ProgressCallback progressCallback_;    // 进度回调

    // 引脚状态跟踪
    int8_t lastActivePin_;    // 上一个激活的引脚（-1表示无）

    // 私有方法
    void initializeGpioPins();      // 初始化GPIO引脚
    void deinitializeGpioPins();    // 反初始化GPIO引脚
    ContinuityState readPinContinuity(
        uint8_t logicalPin);    // 读取单个引脚导通状态
    void configurePinsForSlot(
        uint8_t activePin, bool isActive);  // 为指定时隙配置引脚模式
    void delayMs(uint32_t ms);      // 延迟函数

    // HAL库GPIO辅助函数
    void halGpioInit(const GpioPin& gpioPin, uint32_t mode, uint32_t pull,
                     GPIO_PinState initialState = GPIO_PIN_RESET);
    void halGpioDeinit(const GpioPin& gpioPin);
    GPIO_PinState halGpioRead(const GpioPin& gpioPin);
    void halGpioWrite(const GpioPin& gpioPin, GPIO_PinState state);

   public:
    ContinuityCollector();
    ~ContinuityCollector();

    // 删除拷贝构造函数和赋值操作符
    ContinuityCollector(const ContinuityCollector&) = delete;
    ContinuityCollector& operator=(const ContinuityCollector&) = delete;

    // 配置采集参数
    bool configure(const CollectorConfig& config);

    // 开始采集
    bool startCollection();

    // 停止采集
    void stopCollection();

    // 处理时隙事件（由外部时隙管理器调用）
    void processSlot(uint16_t slotNumber, uint8_t activePin, bool isActive);

    // 获取采集状态
    CollectionStatus getStatus() const;

    // 获取当前周期
    uint16_t getCurrentCycle() const;

    // 获取总周期数
    uint16_t getTotalCycles() const;

    // 获取采集数据
    ContinuityMatrix getDataMatrix() const;

    // 获取指定周期的数据
    std::vector<ContinuityState> getCycleData(uint16_t cycle) const;

    // 检查是否有新数据
    bool hasNewData() const;

    // 检查采集是否完成
    bool isCollectionComplete() const;

    // 设置进度回调
    void setProgressCallback(ProgressCallback callback);

    // 获取采集配置
    const CollectorConfig& getConfig() const { return config_; }

    // 获取采集进度百分比
    float getProgress() const;

    // 获取压缩数据向量（按位压缩，小端模式）
    std::vector<uint8_t> getDataVector() const;

    // 获取指定引脚的所有周期数据
    std::vector<ContinuityState> getPinData(uint8_t pin) const;

    // 清空数据矩阵
    void clearData();

    // 统计功能
    struct Statistics {
        uint32_t totalConnections;       // 总导通次数
        uint32_t totalDisconnections;    // 总断开次数
        double connectionRate;           // 导通率
        uint8_t mostActivePins[5];       // 最活跃的5个引脚
    };

    Statistics calculateStatistics() const;
};

// 导通数据采集器工厂类
class ContinuityCollectorFactory {
   public:
    // 创建采集器
    static std::unique_ptr<ContinuityCollector> create();
};

#endif    // CONTINUITY_COLLECTOR_H