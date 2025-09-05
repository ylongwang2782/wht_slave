#ifndef WHTS_PROTOCOL_COMMON_H
#define WHTS_PROTOCOL_COMMON_H

#include <cstdint>

namespace WhtsProtocol {

// 协议常量
constexpr uint8_t FRAME_DELIMITER_1 = 0xAB;
constexpr uint8_t FRAME_DELIMITER_2 = 0xCD;
constexpr uint32_t BROADCAST_ID = 0xFFFFFFFF;

// Packet ID 枚举
enum class PacketId : uint8_t {
    MASTER_TO_SLAVE = 0x00,
    SLAVE_TO_MASTER = 0x01,
    BACKEND_TO_MASTER = 0x02,
    MASTER_TO_BACKEND = 0x03,
    SLAVE_TO_BACKEND = 0x04
};

// Master2Slave Message ID 枚举
enum class Master2SlaveMessageId : uint8_t {
    SYNC_MSG = 0x00,
    PING_REQ_MSG = 0x40,
    SHORT_ID_ASSIGN_MSG = 0x50,
};

// Slave2Master Message ID 枚举
enum class Slave2MasterMessageId : uint8_t {
    RST_RSP_MSG = 0x30,
    PING_RSP_MSG = 0x41,
    JOIN_REQUEST_MSG = 0x50,
    SHORT_ID_CONFIRM_MSG = 0x51,
    HEARTBEAT_MSG = 0x52,
};

// Backend2Master Message ID 枚举
enum class Backend2MasterMessageId : uint8_t {
    SLAVE_CFG_MSG = 0x00,
    MODE_CFG_MSG = 0x01,
    SLAVE_RST_MSG = 0x02,
    CTRL_MSG = 0x03,
    INTERVAL_CFG_MSG = 0x06,
    PING_CTRL_MSG = 0x10,
    DEVICE_LIST_REQ_MSG = 0x11
};

// Master2Backend Message ID 枚举
enum class Master2BackendMessageId : uint8_t {
    SLAVE_CFG_RSP_MSG = 0x00,
    MODE_CFG_RSP_MSG = 0x01,
    RST_RSP_MSG = 0x02,
    CTRL_RSP_MSG = 0x03,
    PING_RES_MSG = 0x04,
    DEVICE_LIST_RSP_MSG = 0x05,
    INTERVAL_CFG_RSP_MSG = 0x06
};

// Slave2Backend Message ID 枚举
enum class Slave2BackendMessageId : uint8_t {
    CONDUCTION_DATA_MSG = 0x00,
    RESISTANCE_DATA_MSG = 0x01,
    CLIP_DATA_MSG = 0x02
};

}    // namespace WhtsProtocol

#endif    // WHTS_PROTOCOL_COMMON_H