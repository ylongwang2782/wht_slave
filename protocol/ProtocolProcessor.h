#ifndef PROTOCOL_PROCESSOR_H
#define PROTOCOL_PROCESSOR_H

#include "Common.h"
#include "DeviceStatus.h"
#include "Frame.h"
#include "messages/Message.h"
#include <cstdint>
#include <map>
#include <memory>
#include <queue>
#include <vector>

namespace WhtsProtocol {

// 分片相关的私有结构
struct FragmentInfo {
    uint8_t packetId;
    uint32_t sourceId;
    uint8_t totalFragments;
    std::map<uint8_t, std::vector<uint8_t>> fragments; // 序号 -> 分片数据
    uint64_t timestamp;                                // 用于超时处理

    bool isComplete() const { return fragments.size() == totalFragments; }
};

// 协议处理器类
class ProtocolProcessor {
  public:
    ProtocolProcessor();
    ~ProtocolProcessor();

    // 设置最大传输单元大小 (MTU)
    void SetMTU(size_t mtu) { mtu_ = mtu; }
    size_t getMTU() const { return mtu_; }

    // 打包Master2Slave消息 (支持自动分片)
    std::vector<std::vector<uint8_t>>
    packMaster2SlaveMessage(uint32_t destinationId, const Message &message);

    // 打包Slave2Master消息 (支持自动分片)
    std::vector<std::vector<uint8_t>>
    packSlave2MasterMessage(uint32_t slaveId, const Message &message);

    // 打包Slave2Backend消息 (支持自动分片)
    std::vector<std::vector<uint8_t>>
    packSlave2BackendMessage(uint32_t slaveId, const DeviceStatus &deviceStatus,
                             const Message &message);

    // 打包Backend2Master消息 (支持自动分片)
    std::vector<std::vector<uint8_t>>
    packBackend2MasterMessage(const Message &message);

    // 打包Master2Backend消息 (支持自动分片)
    std::vector<std::vector<uint8_t>>
    packMaster2BackendMessage(const Message &message);

    // 兼容旧接口 - 单帧打包
    std::vector<uint8_t> packMaster2SlaveMessageSingle(
        uint32_t destinationId, const Message &message,
        uint8_t fragmentsSequence = 0, uint8_t moreFragmentsFlag = 0);

    std::vector<uint8_t>
    packSlave2MasterMessageSingle(uint32_t slaveId, const Message &message,
                                  uint8_t fragmentsSequence = 0,
                                  uint8_t moreFragmentsFlag = 0);

    std::vector<uint8_t> packSlave2BackendMessageSingle(
        uint32_t slaveId, const DeviceStatus &deviceStatus,
        const Message &message, uint8_t fragmentsSequence = 0,
        uint8_t moreFragmentsFlag = 0);

    std::vector<uint8_t>
    packBackend2MasterMessageSingle(const Message &message,
                                    uint8_t fragmentsSequence = 0,
                                    uint8_t moreFragmentsFlag = 0);

    std::vector<uint8_t>
    packMaster2BackendMessageSingle(const Message &message,
                                    uint8_t fragmentsSequence = 0,
                                    uint8_t moreFragmentsFlag = 0);

    // 处理接收到的原始数据 (支持粘包处理)
    void processReceivedData(const std::vector<uint8_t> &data);

    // 获取完整的已解析帧
    bool getNextCompleteFrame(Frame &frame);

    // 清空接收缓冲区
    void clearReceiveBuffer();

    // 解析单个帧
    bool parseFrame(const std::vector<uint8_t> &data, Frame &frame);

    // 根据Packet ID和Message ID创建对应的消息对象
    std::unique_ptr<Message> createMessage(PacketId packetId,
                                           uint8_t messageId);

    // 解析Master2Slave包
    bool parseMaster2SlavePacket(const std::vector<uint8_t> &payload,
                                 uint32_t &destinationId,
                                 std::unique_ptr<Message> &message);

    // 解析Slave2Master包
    bool parseSlave2MasterPacket(const std::vector<uint8_t> &payload,
                                 uint32_t &slaveId,
                                 std::unique_ptr<Message> &message);

    // 解析Slave2Backend包
    bool parseSlave2BackendPacket(const std::vector<uint8_t> &payload,
                                  uint32_t &slaveId, DeviceStatus &deviceStatus,
                                  std::unique_ptr<Message> &message);

    // 解析Backend2Master包
    bool parseBackend2MasterPacket(const std::vector<uint8_t> &payload,
                                   std::unique_ptr<Message> &message);

    // 解析Master2Backend包
    bool parseMaster2BackendPacket(const std::vector<uint8_t> &payload,
                                   std::unique_ptr<Message> &message);

  private:
    // 帧分片
    std::vector<std::vector<uint8_t>>
    fragmentFrame(const std::vector<uint8_t> &frameData);

    // 分片重组
    bool reassembleFragments(const Frame &frame,
                             std::vector<uint8_t> &completeFrame);

    // 从接收缓冲区中提取完整帧
    bool extractCompleteFrames();

    // 查找帧头
    size_t findFrameHeader(const std::vector<uint8_t> &buffer, size_t startPos);

    // 工具函数
    void writeUint16LE(std::vector<uint8_t> &buffer, uint16_t value);
    void writeUint32LE(std::vector<uint8_t> &buffer, uint32_t value);
    uint16_t readUint16LE(const std::vector<uint8_t> &buffer, size_t offset);
    uint32_t readUint32LE(const std::vector<uint8_t> &buffer, size_t offset);

    // 生成分片的唯一ID
    uint64_t generateFragmentId(uint8_t packetId);

    // 清理超时的分片
    void cleanupExpiredFragments();

  private:
    size_t mtu_;                         // 最大传输单元大小，默认100字节
    std::vector<uint8_t> receiveBuffer_; // 接收缓冲区
    std::queue<Frame> completeFrames_;   // 完整帧队列
    std::map<uint64_t, FragmentInfo> fragmentMap_; // 分片重组映射

    static constexpr uint32_t FRAGMENT_TIMEOUT_MS =
        5000;                                  // 分片超时时间（毫秒）
    static constexpr size_t DEFAULT_MTU = 100; // 默认MTU大小
    static constexpr size_t MAX_RECEIVE_BUFFER_SIZE =
        4096; // 最大接收缓冲区大小
};

} // namespace WhtsProtocol

#endif // PROTOCOL_PROCESSOR_H