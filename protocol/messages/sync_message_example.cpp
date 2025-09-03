/**
 * @file sync_message_example.cpp
 * @brief 新的SyncMessage使用示例
 * 
 * 本文件演示了如何使用新的合并SyncMessage来替代原来的轮询机制
 * 新的SyncMessage集成了以下功能：
 * - 时间同步 (原SetTimeMessage)
 * - 从机控制 (原SlaveControlMessage) 
 * - 从机配置 (原ConfigMessage)
 * - 同步广播 (原SyncMessage)
 */

#include "Master2Slave.h"
#include <iostream>
#include <iomanip>

using namespace WhtsProtocol::Master2Slave;

void printSyncMessageExample() {
    std::cout << "\n=== 新的SyncMessage使用示例 ===" << std::endl;
    
    // 创建新的SyncMessage
    SyncMessage syncMsg;
    
    // 1. 设置采集模式（替代原SlaveControlMessage的功能）
    syncMsg.mode = 0;  // 0: 导通检测, 1: 阻值检测, 2: 卡钉检测
    
    // 2. 设置采集间隔
    syncMsg.interval = 100;  // 100ms间隔
    
    // 3. 设置当前时间（替代原SetTimeMessage的功能）
    syncMsg.currentTime = 1700000000000000ULL;  // 微秒时间戳
    
    // 4. 设置启动时间（替代原SlaveControlMessage的启动时间）
    syncMsg.startTime = syncMsg.currentTime + 5000000ULL;  // 5秒后启动
    
    // 5. 配置所有从机（替代原ConfigMessage的功能）
    SlaveConfig slave1;
    slave1.slaveId = 0x12345678;
    slave1.timeSlot = 0;
    slave1.testCount = 8;  // 导通检测数量
    syncMsg.slaveConfigs.push_back(slave1);
    
    SlaveConfig slave2;
    slave2.slaveId = 0x87654321;
    slave2.timeSlot = 1;
    slave2.testCount = 6;
    syncMsg.slaveConfigs.push_back(slave2);
    
    SlaveConfig slave3;
    slave3.slaveId = 0xABCDEF00;
    slave3.timeSlot = 2;
    slave3.testCount = 4;
    syncMsg.slaveConfigs.push_back(slave3);
    
    // 序列化消息
    std::vector<uint8_t> serialized = syncMsg.serialize();
    
    std::cout << "SyncMessage 内容:" << std::endl;
    std::cout << "  模式: " << static_cast<int>(syncMsg.mode) 
              << " (0:导通, 1:阻值, 2:卡钉)" << std::endl;
    std::cout << "  间隔: " << static_cast<int>(syncMsg.interval) << " ms" << std::endl;
    std::cout << "  当前时间: " << syncMsg.currentTime << " us" << std::endl;
    std::cout << "  启动时间: " << syncMsg.startTime << " us" << std::endl;
    std::cout << "  从机数量: " << syncMsg.slaveConfigs.size() << std::endl;
    
    for (size_t i = 0; i < syncMsg.slaveConfigs.size(); ++i) {
        const auto& config = syncMsg.slaveConfigs[i];
        std::cout << "    从机" << (i+1) << ": ID=0x" 
                  << std::hex << config.slaveId << std::dec
                  << ", 时隙=" << static_cast<int>(config.timeSlot)
                  << ", 检测数量=" << static_cast<int>(config.testCount) << std::endl;
    }
    
    std::cout << "序列化后大小: " << serialized.size() << " 字节" << std::endl;
    
    // 打印序列化数据的十六进制表示
    std::cout << "序列化数据: ";
    for (size_t i = 0; i < std::min(serialized.size(), size_t(32)); ++i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') 
                  << static_cast<int>(serialized[i]) << " ";
    }
    if (serialized.size() > 32) {
        std::cout << "... (共" << std::dec << serialized.size() << "字节)";
    }
    std::cout << std::dec << std::endl;
    
    // 测试反序列化
    SyncMessage deserializedMsg;
    if (deserializedMsg.deserialize(serialized)) {
        std::cout << "反序列化成功！" << std::endl;
        std::cout << "验证: 模式=" << static_cast<int>(deserializedMsg.mode)
                  << ", 从机数量=" << deserializedMsg.slaveConfigs.size() << std::endl;
    } else {
        std::cout << "反序列化失败！" << std::endl;
    }
}

void compareOldVsNew() {
    std::cout << "\n=== 新旧消息对比 ===" << std::endl;
    
    std::cout << "旧的轮询机制需要多个消息:" << std::endl;
    std::cout << "1. SyncMessage (仅时间戳)" << std::endl;
    std::cout << "2. SetTimeMessage (时间校准)" << std::endl;
    std::cout << "3. SlaveControlMessage (启动控制)" << std::endl;
    std::cout << "4. ConductionConfigMessage (导通配置)" << std::endl;
    std::cout << "5. ResistanceConfigMessage (阻值配置)" << std::endl;
    std::cout << "6. ClipConfigMessage (卡钉配置)" << std::endl;
    std::cout << "总计: 6个单独的消息，需要多次轮询" << std::endl;
    
    std::cout << "\n新的时分多址机制只需要一个消息:" << std::endl;
    std::cout << "1. 新的SyncMessage (包含所有配置和控制信息)" << std::endl;
    std::cout << "   - 采集模式 (mode)" << std::endl;
    std::cout << "   - 采集间隔 (interval)" << std::endl;
    std::cout << "   - 当前时间 (currentTime)" << std::endl;
    std::cout << "   - 启动时间 (startTime)" << std::endl;
    std::cout << "   - 所有从机配置 (slaveConfigs)" << std::endl;
    std::cout << "总计: 1个合并消息，定时广播" << std::endl;
    
    std::cout << "\n优势:" << std::endl;
    std::cout << "- 严格按照时分多址概念设计" << std::endl;
    std::cout << "- 减少网络通信开销" << std::endl;
    std::cout << "- 简化协议状态机" << std::endl;
    std::cout << "- 提高系统同步性" << std::endl;
    std::cout << "- 支持延迟启动和精确时间控制" << std::endl;
}

// 如果需要编译为独立程序进行测试
#ifdef SYNC_MESSAGE_EXAMPLE_MAIN
int main() {
    printSyncMessageExample();
    compareOldVsNew();
    return 0;
}
#endif
