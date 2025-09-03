# SyncMessage 重构文档

## 概述

本次重构将原有的轮询机制改为严格的时分多址（TDMA）机制，将多个独立的消息合并到一个新的 SyncMessage 中，实现了更高效和同步的通信协议。

## 重构背景

原系统存在以下问题：
1. 使用轮询机制，不符合时分多址的设计理念
2. 需要多个独立消息：SetTimeMessage、SlaveControlMessage、ConfigMessage 等
3. 通信开销大，同步性差
4. 协议状态机复杂

## 新的设计理念

### 时分多址机制
- 主机定时广播同步消息
- 从机严格按照时隙进行数据传输
- 所有配置和控制信息在一个消息中传递
- 支持精确的时间同步和延迟启动

## 新的 SyncMessage 结构

```cpp
class SyncMessage : public Message {
public:
    uint8_t mode;               // 采集模式：0-导通检测，1-阻值检测，2-卡钉检测
    uint8_t interval;           // 采集间隔（ms）
    uint64_t currentTime;       // 当前时间戳（微秒）
    uint64_t startTime;         // 启动时间戳（微秒）
    std::vector<SlaveConfig> slaveConfigs;  // 所有从机配置
};

struct SlaveConfig {
    uint32_t slaveId;       // 4字节从机ID
    uint8_t timeSlot;       // 分配的时隙
    uint8_t testCount;      // 检测数量（导通/阻值/卡钉数量）
};
```

### 消息结构详细说明

| 字段 | 类型 | 大小 | 描述 |
|------|------|------|------|
| Mode | u8 | 1 Byte | 0：导通检测，1：阻值检测，2：卡钉检测 |
| Interval | u8 | 1 Byte | 采集间隔（ms） |
| Current Time | uint64_t | 8 Byte | 时间戳（微秒） |
| Start Time | uint64_t | 8 Byte | 时间戳（微秒） |
| Slave 1 ID | u8[4] | 4 Byte | 4个字节的从机ID |
| Slave 1 Time Slot | u8 | 1 Byte | 为从节点分配的时隙 |
| Slave 1 Test Count | u8 | 1 Byte | 导通检测数量/阻值检测数量/卡钉检测数量 |
| Slave 2 ID | u8[4] | 4 Byte | 4个字节的从机ID |
| Slave 2 Time Slot | u8 | 1 Byte | 为从节点分配的时隙 |
| Slave 2 Test Count | u8 | 1 Byte | 导通检测数量/阻值检测数量/卡钉检测数量 |
| ... | ... | ... | 其他从机配置 |

## 合并的功能

### 1. 原 SyncMessage 功能
- **旧功能**: 仅包含时间戳用于同步
- **新功能**: 完整的系统同步和配置信息

### 2. 原 SetTimeMessage 功能
- **旧功能**: 独立的时间设置消息
- **新功能**: 集成在 `currentTime` 字段中

### 3. 原 SlaveControlMessage 功能
- **旧功能**: 独立的从机控制消息（启动/停止）
- **新功能**: 集成在 `mode` 和 `startTime` 字段中

### 4. 原 ConfigMessage 功能
- **旧功能**: 独立的配置消息（导通、阻值、卡钉配置）
- **新功能**: 集成在 `slaveConfigs` 数组中

## 工作流程

### 主机端流程
1. 上位机发送启动指令
2. 主机开始定时广播 SyncMessage
3. SyncMessage 包含：
   - 当前采集模式
   - 采集间隔
   - 当前时间（用于时间校准）
   - 启动时间（延迟启动控制）
   - 所有从机的配置信息

### 从机端流程
1. 接收 SyncMessage
2. 根据 `currentTime` 进行时间校准
3. 存储采集间隔和模式
4. 根据 `slaveId` 匹配找到自己的配置
5. 设置时隙和检测数量
6. 根据 `startTime` 决定立即启动或延迟启动
7. 启动数据采集，完成后停止
8. 等待下一次 SyncMessage

## 代码变更

### 协议层变更
- `protocol/messages/Master2Slave.h`: 更新 SyncMessage 结构
- `protocol/messages/Master2Slave.cpp`: 实现新的序列化/反序列化

### 从机应用层变更
- `User/app/slave_device.h`: 添加新的配置结构体
- `User/app/master_slave_message_handlers.cpp`: 更新 SyncMessage 处理逻辑

### 兼容性处理
- 旧的消息类型标记为 DEPRECATED
- 保留旧的处理器以确保兼容性
- 新系统优先使用新的 SyncMessage

## 优势

1. **严格时分多址**: 符合 TDMA 设计理念
2. **减少通信开销**: 6个消息合并为1个
3. **提高同步性**: 统一的时间基准和启动控制
4. **简化协议**: 减少状态机复杂性
5. **精确控制**: 支持微秒级时间控制和延迟启动
6. **扩展性好**: 易于添加新的从机和配置

## 测试和验证

- 创建了 `sync_message_example.cpp` 演示新消息的使用
- 验证序列化和反序列化功能
- 确保与现有系统的兼容性

## 向后兼容

为了确保系统的平滑过渡：
1. 保留了所有旧的消息类型定义
2. 旧的消息处理器仍然可以工作
3. 添加了 DEPRECATED 标记提醒开发者
4. 新系统优先处理新的 SyncMessage

## 部署建议

1. **阶段1**: 部署新的从机固件，支持新 SyncMessage
2. **阶段2**: 更新主机端，开始发送新 SyncMessage
3. **阶段3**: 验证系统稳定性
4. **阶段4**: 移除对旧消息的支持（可选）

## 注意事项

1. 确保所有从机都更新到支持新 SyncMessage 的固件
2. 主机端需要相应更新以发送新格式的 SyncMessage
3. 时间同步精度依赖于网络延迟的一致性
4. 从机数量增加时，SyncMessage 大小会相应增长

## 总结

本次重构成功地将原有的轮询机制改为时分多址机制，大幅简化了协议复杂性，提高了系统的同步性和效率。新的 SyncMessage 集成了所有必要的配置和控制信息，为系统的进一步优化奠定了基础。
