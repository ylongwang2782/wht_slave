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

// 使用CubeMX生成的宏定义的引脚映射表 (64个引脚)
static const struct {
    GPIO_TypeDef* port;
    uint16_t pin;
} HARDWARE_PIN_MAP[64] = {
    // IO1-IO64 使用CubeMX生成的宏定义
    {IO1_GPIO_Port, IO1_Pin},      // IO1
    {IO2_GPIO_Port, IO2_Pin},      // IO2
    {IO3_GPIO_Port, IO3_Pin},      // IO3
    {IO4_GPIO_Port, IO4_Pin},      // IO4
    {IO5_GPIO_Port, IO5_Pin},      // IO5
    {IO6_GPIO_Port, IO6_Pin},      // IO6
    {IO7_GPIO_Port, IO7_Pin},      // IO7
    {IO8_GPIO_Port, IO8_Pin},      // IO8
    {IO9_GPIO_Port, IO9_Pin},      // IO9
    {IO10_GPIO_Port, IO10_Pin},    // IO10
    {IO11_GPIO_Port, IO11_Pin},    // IO11
    {IO12_GPIO_Port, IO12_Pin},    // IO12
    {IO13_GPIO_Port, IO13_Pin},    // IO13
    {IO14_GPIO_Port, IO14_Pin},    // IO14
    {IO15_GPIO_Port, IO15_Pin},    // IO15
    {IO16_GPIO_Port, IO16_Pin},    // IO16
    {IO17_GPIO_Port, IO17_Pin},    // IO17
    {IO18_GPIO_Port, IO18_Pin},    // IO18
    {IO19_GPIO_Port, IO19_Pin},    // IO19
    {IO20_GPIO_Port, IO20_Pin},    // IO20
    {IO21_GPIO_Port, IO21_Pin},    // IO21
    {IO22_GPIO_Port, IO22_Pin},    // IO22
    {IO23_GPIO_Port, IO23_Pin},    // IO23
    {IO24_GPIO_Port, IO24_Pin},    // IO24
    {IO25_GPIO_Port, IO25_Pin},    // IO25
    {IO26_GPIO_Port, IO26_Pin},    // IO26
    {IO27_GPIO_Port, IO27_Pin},    // IO27
    {IO28_GPIO_Port, IO28_Pin},    // IO28
    {IO29_GPIO_Port, IO29_Pin},    // IO29
    {IO30_GPIO_Port, IO30_Pin},    // IO30
    {IO31_GPIO_Port, IO31_Pin},    // IO31
    {IO32_GPIO_Port, IO32_Pin},    // IO32
    {IO33_GPIO_Port, IO33_Pin},    // IO33
    {IO34_GPIO_Port, IO34_Pin},    // IO34
    {IO35_GPIO_Port, IO35_Pin},    // IO35
    {IO36_GPIO_Port, IO36_Pin},    // IO36
    {IO37_GPIO_Port, IO37_Pin},    // IO37
    {IO38_GPIO_Port, IO38_Pin},    // IO38
    {IO39_GPIO_Port, IO39_Pin},    // IO39
    {IO40_GPIO_Port, IO40_Pin},    // IO40
    {IO41_GPIO_Port, IO41_Pin},    // IO41
    {IO42_GPIO_Port, IO42_Pin},    // IO42
    {IO43_GPIO_Port, IO43_Pin},    // IO43
    {IO44_GPIO_Port, IO44_Pin},    // IO44
    {IO45_GPIO_Port, IO45_Pin},    // IO45
    {IO46_GPIO_Port, IO46_Pin},    // IO46
    {IO47_GPIO_Port, IO47_Pin},    // IO47
    {IO48_GPIO_Port, IO48_Pin},    // IO48
    {IO49_GPIO_Port, IO49_Pin},    // IO49
    {IO50_GPIO_Port, IO50_Pin},    // IO50
    {IO51_GPIO_Port, IO51_Pin},    // IO51
    {IO52_GPIO_Port, IO52_Pin},    // IO52
    {IO53_GPIO_Port, IO53_Pin},    // IO53
    {IO54_GPIO_Port, IO54_Pin},    // IO54
    {IO55_GPIO_Port, IO55_Pin},    // IO55
    {IO56_GPIO_Port, IO56_Pin},    // IO56
    {IO57_GPIO_Port, IO57_Pin},    // IO57
    {IO58_GPIO_Port, IO58_Pin},    // IO58
    {IO59_GPIO_Port, IO59_Pin},    // IO59
    {IO60_GPIO_Port, IO60_Pin},    // IO60
    {IO61_GPIO_Port, IO61_Pin},    // IO61
    {IO62_GPIO_Port, IO62_Pin},    // IO62
    {IO63_GPIO_Port, IO63_Pin},    // IO63
    {IO64_GPIO_Port, IO64_Pin}     // IO64
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
            // 现在直接返回逻辑引脚号，因为映射表现在直接包含GPIO信息
            elog_v("CollectorConfig", "returning logicalPin: %d", logicalPin);
            return logicalPin;
        }

        elog_v("CollectorConfig", "logicalPin out of range, returning: %d",
               logicalPin);
        return logicalPin;    // 如果超出范围，返回逻辑引脚号
    }

    // 将逻辑引脚号转换为GPIO端口和引脚
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