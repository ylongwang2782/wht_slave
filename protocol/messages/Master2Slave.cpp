#include "Master2Slave.h"

namespace WhtsProtocol {
namespace Master2Slave {

// SyncMessage 实现
std::vector<uint8_t> SyncMessage::serialize() const {
    std::vector<uint8_t> result;
    
    // 序列化模式（1字节）
    result.push_back(mode);
    
    // 序列化间隔（1字节）
    result.push_back(interval);
    
    // 序列化当前时间戳（8字节，小端序）
    result.push_back(currentTime & 0xFF);
    result.push_back((currentTime >> 8) & 0xFF);
    result.push_back((currentTime >> 16) & 0xFF);
    result.push_back((currentTime >> 24) & 0xFF);
    result.push_back((currentTime >> 32) & 0xFF);
    result.push_back((currentTime >> 40) & 0xFF);
    result.push_back((currentTime >> 48) & 0xFF);
    result.push_back((currentTime >> 56) & 0xFF);
    
    // 序列化启动时间戳（8字节，小端序）
    result.push_back(startTime & 0xFF);
    result.push_back((startTime >> 8) & 0xFF);
    result.push_back((startTime >> 16) & 0xFF);
    result.push_back((startTime >> 24) & 0xFF);
    result.push_back((startTime >> 32) & 0xFF);
    result.push_back((startTime >> 40) & 0xFF);
    result.push_back((startTime >> 48) & 0xFF);
    result.push_back((startTime >> 56) & 0xFF);
    
    // 序列化从机配置
    for (const auto& config : slaveConfigs) {
        // 从机ID（4字节，小端序）
        result.push_back(config.slaveId & 0xFF);
        result.push_back((config.slaveId >> 8) & 0xFF);
        result.push_back((config.slaveId >> 16) & 0xFF);
        result.push_back((config.slaveId >> 24) & 0xFF);
        
        // 时隙（1字节）
        result.push_back(config.timeSlot);
        
        // 检测数量（1字节）
        result.push_back(config.testCount);
    }
    
    return result;
}

bool SyncMessage::deserialize(const std::vector<uint8_t> &data) {
    if (data.size() < 18) return false; // 最小长度：1+1+8+8=18字节
    
    size_t offset = 0;
    
    // 反序列化模式
    mode = data[offset++];
    
    // 反序列化间隔
    interval = data[offset++];
    
    // 反序列化当前时间戳（8字节，小端序）
    currentTime = static_cast<uint64_t>(data[offset]) |
                  (static_cast<uint64_t>(data[offset + 1]) << 8) |
                  (static_cast<uint64_t>(data[offset + 2]) << 16) |
                  (static_cast<uint64_t>(data[offset + 3]) << 24) |
                  (static_cast<uint64_t>(data[offset + 4]) << 32) |
                  (static_cast<uint64_t>(data[offset + 5]) << 40) |
                  (static_cast<uint64_t>(data[offset + 6]) << 48) |
                  (static_cast<uint64_t>(data[offset + 7]) << 56);
    offset += 8;
    
    // 反序列化启动时间戳（8字节，小端序）
    startTime = static_cast<uint64_t>(data[offset]) |
                (static_cast<uint64_t>(data[offset + 1]) << 8) |
                (static_cast<uint64_t>(data[offset + 2]) << 16) |
                (static_cast<uint64_t>(data[offset + 3]) << 24) |
                (static_cast<uint64_t>(data[offset + 4]) << 32) |
                (static_cast<uint64_t>(data[offset + 5]) << 40) |
                (static_cast<uint64_t>(data[offset + 6]) << 48) |
                (static_cast<uint64_t>(data[offset + 7]) << 56);
    offset += 8;
    
    // 反序列化从机配置（每个从机配置6字节：4字节ID + 1字节时隙 + 1字节检测数量）
    slaveConfigs.clear();
    while (offset + 6 <= data.size()) {
        SlaveConfig config;
        
        // 从机ID（4字节，小端序）
        config.slaveId = static_cast<uint32_t>(data[offset]) |
                         (static_cast<uint32_t>(data[offset + 1]) << 8) |
                         (static_cast<uint32_t>(data[offset + 2]) << 16) |
                         (static_cast<uint32_t>(data[offset + 3]) << 24);
        offset += 4;
        
        // 时隙
        config.timeSlot = data[offset++];
        
        // 检测数量
        config.testCount = data[offset++];
        
        slaveConfigs.push_back(config);
    }
    
    return true;
}

// SetTimeMessage 实现
std::vector<uint8_t> SetTimeMessage::serialize() const {
    std::vector<uint8_t> result;
    result.push_back(timestamp & 0xFF);
    result.push_back((timestamp >> 8) & 0xFF);
    result.push_back((timestamp >> 16) & 0xFF);
    result.push_back((timestamp >> 24) & 0xFF);
    result.push_back((timestamp >> 32) & 0xFF);
    result.push_back((timestamp >> 40) & 0xFF);
    result.push_back((timestamp >> 48) & 0xFF);
    result.push_back((timestamp >> 56) & 0xFF);
    return result;
}

bool SetTimeMessage::deserialize(const std::vector<uint8_t> &data) {
    if (data.size() < 8) return false;
    // 反序列化8字节时间戳（小端序）
    timestamp = static_cast<uint64_t>(data[0]) |
                (static_cast<uint64_t>(data[1]) << 8) |
                (static_cast<uint64_t>(data[2]) << 16) |
                (static_cast<uint64_t>(data[3]) << 24) |
                (static_cast<uint64_t>(data[4]) << 32) |
                (static_cast<uint64_t>(data[5]) << 40) |
                (static_cast<uint64_t>(data[6]) << 48) |
                (static_cast<uint64_t>(data[7]) << 56);
    return true;
}

// ConductionConfigMessage 实现
std::vector<uint8_t> ConductionConfigMessage::serialize() const {
    std::vector<uint8_t> result;
    result.push_back(timeSlot);
    result.push_back(interval);
    result.push_back(totalConductionNum & 0xFF);
    result.push_back((totalConductionNum >> 8) & 0xFF);
    result.push_back(startConductionNum & 0xFF);
    result.push_back((startConductionNum >> 8) & 0xFF);
    result.push_back(conductionNum & 0xFF);
    result.push_back((conductionNum >> 8) & 0xFF);
    return result;
}

bool ConductionConfigMessage::deserialize(const std::vector<uint8_t> &data) {
    if (data.size() < 8) return false;
    timeSlot = data[0];
    interval = data[1];
    totalConductionNum = data[2] | (data[3] << 8);
    startConductionNum = data[4] | (data[5] << 8);
    conductionNum = data[6] | (data[7] << 8);
    return true;
}

// ResistanceConfigMessage 实现
std::vector<uint8_t> ResistanceConfigMessage::serialize() const {
    std::vector<uint8_t> result;
    result.push_back(timeSlot);
    result.push_back(interval);
    result.push_back(totalNum & 0xFF);
    result.push_back((totalNum >> 8) & 0xFF);
    result.push_back(startNum & 0xFF);
    result.push_back((startNum >> 8) & 0xFF);
    result.push_back(num & 0xFF);
    result.push_back((num >> 8) & 0xFF);
    return result;
}

bool ResistanceConfigMessage::deserialize(const std::vector<uint8_t> &data) {
    if (data.size() < 8) return false;
    timeSlot = data[0];
    interval = data[1];
    totalNum = data[2] | (data[3] << 8);
    startNum = data[4] | (data[5] << 8);
    num = data[6] | (data[7] << 8);
    return true;
}

// ClipConfigMessage 实现
std::vector<uint8_t> ClipConfigMessage::serialize() const {
    std::vector<uint8_t> result;
    result.push_back(interval);
    result.push_back(mode);
    result.push_back(clipPin & 0xFF);
    result.push_back((clipPin >> 8) & 0xFF);
    return result;
}

bool ClipConfigMessage::deserialize(const std::vector<uint8_t> &data) {
    if (data.size() < 4) return false;
    interval = data[0];
    mode = data[1];
    clipPin = data[2] | (data[3] << 8);
    return true;
}

// RstMessage 实现
std::vector<uint8_t> RstMessage::serialize() const {
    std::vector<uint8_t> result;
    result.push_back(lockStatus);
    result.push_back(clipLed & 0xFF);
    result.push_back((clipLed >> 8) & 0xFF);
    return result;
}

bool RstMessage::deserialize(const std::vector<uint8_t> &data) {
    if (data.size() < 3) return false;
    lockStatus = data[0];
    clipLed = data[1] | (data[2] << 8);
    return true;
}

// PingReqMessage 实现
std::vector<uint8_t> PingReqMessage::serialize() const {
    std::vector<uint8_t> result;
    result.push_back(sequenceNumber & 0xFF);
    result.push_back((sequenceNumber >> 8) & 0xFF);
    result.push_back(timestamp & 0xFF);
    result.push_back((timestamp >> 8) & 0xFF);
    result.push_back((timestamp >> 16) & 0xFF);
    result.push_back((timestamp >> 24) & 0xFF);
    return result;
}

bool PingReqMessage::deserialize(const std::vector<uint8_t> &data) {
    if (data.size() < 6) return false;
    sequenceNumber = data[0] | (data[1] << 8);
    timestamp = data[2] | (data[3] << 8) | (data[4] << 16) | (data[5] << 24);
    return true;
}

// ShortIdAssignMessage 实现
std::vector<uint8_t> ShortIdAssignMessage::serialize() const {
    return {shortId};
}

bool ShortIdAssignMessage::deserialize(const std::vector<uint8_t> &data) {
    if (data.size() < 1) return false;
    shortId = data[0];
    return true;
}

// SlaveControlMessage 实现
std::vector<uint8_t> SlaveControlMessage::serialize() const {
    std::vector<uint8_t> result;
    result.push_back(static_cast<uint8_t>(mode));
    result.push_back(enable);
    // 序列化8字节启动时间戳（小端序）
    result.push_back(startTime & 0xFF);
    result.push_back((startTime >> 8) & 0xFF);
    result.push_back((startTime >> 16) & 0xFF);
    result.push_back((startTime >> 24) & 0xFF);
    result.push_back((startTime >> 32) & 0xFF);
    result.push_back((startTime >> 40) & 0xFF);
    result.push_back((startTime >> 48) & 0xFF);
    result.push_back((startTime >> 56) & 0xFF);
    return result;
}

bool SlaveControlMessage::deserialize(const std::vector<uint8_t> &data) {
    if (data.size() < 10) return false;  // 1 + 1 + 8 = 10 bytes
    mode = static_cast<SlaveRunMode>(data[0]);
    enable = data[1];
    // 反序列化8字节启动时间戳（小端序）
    startTime = static_cast<uint64_t>(data[2]) |
                (static_cast<uint64_t>(data[3]) << 8) |
                (static_cast<uint64_t>(data[4]) << 16) |
                (static_cast<uint64_t>(data[5]) << 24) |
                (static_cast<uint64_t>(data[6]) << 32) |
                (static_cast<uint64_t>(data[7]) << 40) |
                (static_cast<uint64_t>(data[8]) << 48) |
                (static_cast<uint64_t>(data[9]) << 56);
    return true;
}

}    // namespace Master2Slave
}    // namespace WhtsProtocol