#ifndef WHTS_PROTOCOL_SLAVE2MASTER_H
#define WHTS_PROTOCOL_SLAVE2MASTER_H

#include "../Common.h"
#include "Message.h"

namespace WhtsProtocol {
namespace Slave2Master {





class RstResponseMessage : public Message {
   public:
    uint8_t status;         // 0：复位成功，1：复位异常

    std::vector<uint8_t> serialize() const override;
    bool deserialize(const std::vector<uint8_t>& data) override;
    uint8_t getMessageId() const override {
        return static_cast<uint8_t>(Slave2MasterMessageId::RST_RSP_MSG);
    }
    const char* getMessageTypeName() const override { return "Reset Response"; }
};

class PingRspMessage : public Message {
   public:
    uint16_t sequenceNumber;
    uint32_t timestamp;

    std::vector<uint8_t> serialize() const override;
    bool deserialize(const std::vector<uint8_t>& data) override;
    uint8_t getMessageId() const override {
        return static_cast<uint8_t>(Slave2MasterMessageId::PING_RSP_MSG);
    }
    const char* getMessageTypeName() const override { return "Ping Response"; }
};

class JoinRequestMessage : public Message {
   public:
    uint32_t deviceId;
    uint8_t versionMajor;
    uint8_t versionMinor;
    uint16_t versionPatch;

    std::vector<uint8_t> serialize() const override;
    bool deserialize(const std::vector<uint8_t>& data) override;
    uint8_t getMessageId() const override {
        return static_cast<uint8_t>(Slave2MasterMessageId::JOIN_REQUEST_MSG);
    }
    const char* getMessageTypeName() const override { return "JoinRequest"; }
};

class ShortIdConfirmMessage : public Message {
   public:
    uint8_t status;
    uint8_t shortId;

    std::vector<uint8_t> serialize() const override;
    bool deserialize(const std::vector<uint8_t>& data) override;
    uint8_t getMessageId() const override {
        return static_cast<uint8_t>(
            Slave2MasterMessageId::SHORT_ID_CONFIRM_MSG);
    }
    const char* getMessageTypeName() const override {
        return "Short ID Confirm";
    }
};

class HeartbeatMessage : public Message {
   public:
    uint8_t batteryLevel;  // 电池电量百分比 (0-100%)

    std::vector<uint8_t> serialize() const override;
    bool deserialize(const std::vector<uint8_t>& data) override;
    uint8_t getMessageId() const override {
        return static_cast<uint8_t>(Slave2MasterMessageId::HEARTBEAT_MSG);
    }
    const char* getMessageTypeName() const override { return "Heartbeat"; }
};


}    // namespace Slave2Master
}    // namespace WhtsProtocol

#endif    // WHTS_PROTOCOL_SLAVE2MASTER_H