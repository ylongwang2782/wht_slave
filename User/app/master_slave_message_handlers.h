#pragma once

#include <functional>
#include <memory>

#include "continuity_collector.h"
#include "slave_device_state.h"
#include "WhtsProtocol.h"


using namespace WhtsProtocol;

namespace SlaveApp {

// Forward declarations
class SlaveDevice;

// Message handler interface for Master2Slave messages
class IMaster2SlaveMessageHandler {
   public:
    virtual ~IMaster2SlaveMessageHandler() = default;
    virtual std::unique_ptr<Message> processMessage(const Message &message,
                                                    SlaveDevice *device) = 0;
};

// Sync Message Handler
class SyncMessageHandler : public IMaster2SlaveMessageHandler {
   public:
    static SyncMessageHandler &getInstance() {
        static SyncMessageHandler instance;
        return instance;
    }
    std::unique_ptr<Message> processMessage(const Message &message,
                                            SlaveDevice *device) override;

   private:
    SyncMessageHandler() = default;
    SyncMessageHandler(const SyncMessageHandler &) = delete;
    SyncMessageHandler &operator=(const SyncMessageHandler &) = delete;
};

// SetTime Message Handler
class SetTimeMessageHandler : public IMaster2SlaveMessageHandler {
   public:
    static SetTimeMessageHandler &getInstance() {
        static SetTimeMessageHandler instance;
        return instance;
    }
    std::unique_ptr<Message> processMessage(const Message &message,
                                            SlaveDevice *device) override;

   private:
    SetTimeMessageHandler() = default;
    SetTimeMessageHandler(const SetTimeMessageHandler &) = delete;
    SetTimeMessageHandler &operator=(const SetTimeMessageHandler &) = delete;
};

// Conduction Config Message Handler
class ConductionConfigHandler : public IMaster2SlaveMessageHandler {
   public:
    static ConductionConfigHandler &getInstance() {
        static ConductionConfigHandler instance;
        return instance;
    }
    std::unique_ptr<Message> processMessage(const Message &message,
                                            SlaveDevice *device) override;

   private:
    ConductionConfigHandler() = default;
    ConductionConfigHandler(const ConductionConfigHandler &) = delete;
    ConductionConfigHandler &operator=(const ConductionConfigHandler &) =
        delete;
};

// Resistance Config Message Handler
class ResistanceConfigHandler : public IMaster2SlaveMessageHandler {
   public:
    static ResistanceConfigHandler &getInstance() {
        static ResistanceConfigHandler instance;
        return instance;
    }
    std::unique_ptr<Message> processMessage(const Message &message,
                                            SlaveDevice *device) override;

   private:
    ResistanceConfigHandler() = default;
    ResistanceConfigHandler(const ResistanceConfigHandler &) = delete;
    ResistanceConfigHandler &operator=(const ResistanceConfigHandler &) =
        delete;
};

// Clip Config Message Handler
class ClipConfigHandler : public IMaster2SlaveMessageHandler {
   public:
    static ClipConfigHandler &getInstance() {
        static ClipConfigHandler instance;
        return instance;
    }
    std::unique_ptr<Message> processMessage(const Message &message,
                                            SlaveDevice *device) override;

   private:
    ClipConfigHandler() = default;
    ClipConfigHandler(const ClipConfigHandler &) = delete;
    ClipConfigHandler &operator=(const ClipConfigHandler &) = delete;
};

// Note: ReadConductionDataHandler, ReadResistanceDataHandler, and
// ReadClipDataHandler have been removed as they conflict with the new
// push-based data collection architecture.

// Ping Request Message Handler
class PingRequestHandler : public IMaster2SlaveMessageHandler {
   public:
    static PingRequestHandler &getInstance() {
        static PingRequestHandler instance;
        return instance;
    }
    std::unique_ptr<Message> processMessage(const Message &message,
                                            SlaveDevice *device) override;

   private:
    PingRequestHandler() = default;
    PingRequestHandler(const PingRequestHandler &) = delete;
    PingRequestHandler &operator=(const PingRequestHandler &) = delete;
};

// Reset Message Handler
class ResetMessageHandler : public IMaster2SlaveMessageHandler {
   public:
    static ResetMessageHandler &getInstance() {
        static ResetMessageHandler instance;
        return instance;
    }
    std::unique_ptr<Message> processMessage(const Message &message,
                                            SlaveDevice *device) override;

   private:
    ResetMessageHandler() = default;
    ResetMessageHandler(const ResetMessageHandler &) = delete;
    ResetMessageHandler &operator=(const ResetMessageHandler &) = delete;
};

// Short ID Assignment Message Handler
class ShortIdAssignHandler : public IMaster2SlaveMessageHandler {
   public:
    static ShortIdAssignHandler &getInstance() {
        static ShortIdAssignHandler instance;
        return instance;
    }
    std::unique_ptr<Message> processMessage(const Message &message,
                                            SlaveDevice *device) override;

   private:
    ShortIdAssignHandler() = default;
    ShortIdAssignHandler(const ShortIdAssignHandler &) = delete;
    ShortIdAssignHandler &operator=(const ShortIdAssignHandler &) = delete;
};

// Slave Control Message Handler
class SlaveControlHandler : public IMaster2SlaveMessageHandler {
   public:
    static SlaveControlHandler &getInstance() {
        static SlaveControlHandler instance;
        return instance;
    }
    std::unique_ptr<Message> processMessage(const Message &message,
                                            SlaveDevice *device) override;

   private:
    SlaveControlHandler() = default;
    SlaveControlHandler(const SlaveControlHandler &) = delete;
    SlaveControlHandler &operator=(const SlaveControlHandler &) = delete;
};

}    // namespace SlaveApp