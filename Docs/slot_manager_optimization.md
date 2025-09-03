# SlotManager 优化文档

## 问题描述

在实现新的时分多址机制时，遇到了以下问题：

```
E/SlotManager     [210697] Cannot configure while running
E/SyncMessageHandler [210698] Failed to configure slot manager
```

**根本原因**: 原有的SlotManager设计为自恢复启动模式，持续运行直到接收到停止指令。但新的时分多址机制需要在每次接收到SyncMessage后启动一个周期，然后停止等待下一次SyncMessage。

## 解决方案

### 1. 添加单周期模式支持

在`SlotManager`中添加了新的配置选项和状态管理：

```cpp
// 新增私有成员变量
bool m_SingleCycleMode;     // 是否为单周期模式
bool m_CycleCompleted;      // 是否已完成一个周期

// 新增配置方法重载
bool Configure(uint16_t startSlot, uint8_t deviceSlotCount, uint16_t totalSlotCount, 
               uint32_t slotIntervalMs, bool singleCycle);
```

### 2. 修改Process方法

在Process方法中添加了单周期完成检测：

```cpp
// 检查单周期模式下是否已完成一个完整周期
if (m_SingleCycleMode && !m_CycleCompleted && totalCycles >= m_TotalSlotCount)
{
    m_CycleCompleted = true;
    elog_i("SlotManager", "Single cycle completed, stopping slot management");
    Stop();
    return;
}
```

### 3. 更新SyncMessageHandler

在接收到SyncMessage时：

1. **先停止**当前运行的SlotManager
2. **重新配置**为单周期模式
3. **启动**新的周期

```cpp
// 先停止当前运行的时隙管理器
device->m_slotManager->Stop();

// 使用单周期模式配置SlotManager
if (!device->m_slotManager->Configure(startSlot, deviceSlotCount, totalSlotCount, slotIntervalMs, true))
```

## 工作流程

### 新的时分多址工作流程

1. **接收SyncMessage**
   - 停止当前运行的SlotManager
   - 进行时间同步
   - 解析从机配置

2. **配置SlotManager**
   - 使用单周期模式配置
   - 设置时隙和间隔参数

3. **启动数据采集**
   - 启动ContinuityCollector
   - 启动SlotManager（单周期模式）

4. **自动停止**
   - SlotManager完成一个完整周期后自动停止
   - 等待下一次SyncMessage

### 日志输出示例

成功的日志输出应该类似：
```
I/SyncMessageHandler [xxx] Processing new sync message - Mode: 0, Interval: 10 ms
I/SyncMessageHandler [xxx] Time sync - Local: xxx us, Master: xxx us, Offset: xxx us
I/SyncMessageHandler [xxx] Found config for device 0x02302189 - TimeSlot: 1, TestCount: 16
I/SyncMessageHandler [xxx] Continuity collector configured successfully
I/SlotManager        [xxx] Configured (single cycle) - StartSlot: 1, TotalSlots: 4, Interval: 10ms, SingleCycle: Yes
I/SyncMessageHandler [xxx] Data collection and slot management started successfully
...
I/SlotManager        [xxx] Single cycle completed, stopping slot management
```

## 优势

1. **严格时分多址**: 每次SyncMessage启动一个完整的数据采集周期
2. **自动管理**: 无需手动控制启动/停止时机
3. **资源节约**: 不采集时SlotManager处于停止状态
4. **同步精确**: 每次都基于最新的时间同步信息启动
5. **向后兼容**: 保留了原有的连续模式配置方法

## 兼容性

- 保留了原有的`Configure()`方法，默认为连续模式
- 新增了支持单周期模式的重载方法
- 旧的配置消息处理器也更新为使用单周期模式

这样的设计确保了新的时分多址机制能够正确工作，同时保持了与现有代码的兼容性。
