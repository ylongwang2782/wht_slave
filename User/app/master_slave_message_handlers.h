#pragma once

#include <memory>

#include "WhtsProtocol.h"

using namespace WhtsProtocol;

namespace SlaveApp
{

// Forward declarations
class SlaveDevice;

// Message handler interface for Master2Slave messages
class IMaster2SlaveMessageHandler
{
  public:
    virtual ~IMaster2SlaveMessageHandler() = default;
    virtual std::unique_ptr<Message> ProcessMessage(const Message &message, SlaveDevice *device) = 0;
};

// Sync Message Handler
class SyncMessageHandler final : public IMaster2SlaveMessageHandler
{
  public:
    static SyncMessageHandler &GetInstance()
    {
        static SyncMessageHandler instance;
        return instance;
    }
    std::unique_ptr<Message> ProcessMessage(const Message &message, SlaveDevice *device) override;
    SyncMessageHandler &operator=(const SyncMessageHandler &) = delete;
    SyncMessageHandler(const SyncMessageHandler &) = delete;

  private:
    SyncMessageHandler() = default;
};

// SetTime Message Handler
class SetTimeMessageHandler final : public IMaster2SlaveMessageHandler
{
  public:
    static SetTimeMessageHandler &GetInstance()
    {
        static SetTimeMessageHandler instance;
        return instance;
    }
    std::unique_ptr<Message> ProcessMessage(const Message &message, SlaveDevice *device) override;
    SetTimeMessageHandler(const SetTimeMessageHandler &) = delete;
    SetTimeMessageHandler &operator=(const SetTimeMessageHandler &) = delete;

  private:
    SetTimeMessageHandler() = default;
};

// Conduction Config Message Handler
class ConductionConfigHandler final : public IMaster2SlaveMessageHandler
{
  public:
    static ConductionConfigHandler &GetInstance()
    {
        static ConductionConfigHandler instance;
        return instance;
    }
    std::unique_ptr<Message> ProcessMessage(const Message &message, SlaveDevice *device) override;
    ConductionConfigHandler(const ConductionConfigHandler &) = delete;
    ConductionConfigHandler &operator=(const ConductionConfigHandler &) = delete;

  private:
    ConductionConfigHandler() = default;
};

// Resistance Config Message Handler
class ResistanceConfigHandler final : public IMaster2SlaveMessageHandler
{
  public:
    static ResistanceConfigHandler &GetInstance()
    {
        static ResistanceConfigHandler instance;
        return instance;
    }
    std::unique_ptr<Message> ProcessMessage(const Message &message, SlaveDevice *device) override;
    ResistanceConfigHandler(const ResistanceConfigHandler &) = delete;
    ResistanceConfigHandler &operator=(const ResistanceConfigHandler &) = delete;

  private:
    ResistanceConfigHandler() = default;
};

// Clip Config Message Handler
class ClipConfigHandler final : public IMaster2SlaveMessageHandler
{
  public:
    static ClipConfigHandler &GetInstance()
    {
        static ClipConfigHandler instance;
        return instance;
    }
    std::unique_ptr<Message> ProcessMessage(const Message &message, SlaveDevice *device) override;
    ClipConfigHandler(const ClipConfigHandler &) = delete;
    ClipConfigHandler &operator=(const ClipConfigHandler &) = delete;

  private:
    ClipConfigHandler() = default;
};

// Ping Request Message Handler
class PingRequestHandler final : public IMaster2SlaveMessageHandler
{
  public:
    static PingRequestHandler &GetInstance()
    {
        static PingRequestHandler instance;
        return instance;
    }
    std::unique_ptr<Message> ProcessMessage(const Message &message, SlaveDevice *device) override;
    PingRequestHandler(const PingRequestHandler &) = delete;
    PingRequestHandler &operator=(const PingRequestHandler &) = delete;

  private:
    PingRequestHandler() = default;
};

// Reset Message Handler
class ResetMessageHandler final : public IMaster2SlaveMessageHandler
{
  public:
    static ResetMessageHandler &GetInstance()
    {
        static ResetMessageHandler instance;
        return instance;
    }
    std::unique_ptr<Message> ProcessMessage(const Message &message, SlaveDevice *device) override;
    ResetMessageHandler(const ResetMessageHandler &) = delete;
    ResetMessageHandler &operator=(const ResetMessageHandler &) = delete;

  private:
    ResetMessageHandler() = default;
};

// Short ID Assignment Message Handler
class ShortIdAssignHandler final : public IMaster2SlaveMessageHandler
{
  public:
    static ShortIdAssignHandler &GetInstance()
    {
        static ShortIdAssignHandler instance;
        return instance;
    }
    std::unique_ptr<Message> ProcessMessage(const Message &message, SlaveDevice *device) override;
    ShortIdAssignHandler(const ShortIdAssignHandler &) = delete;
    ShortIdAssignHandler &operator=(const ShortIdAssignHandler &) = delete;

  private:
    ShortIdAssignHandler() = default;
};

// Secondary Control Message Handler
class SlaveControlHandler final : public IMaster2SlaveMessageHandler
{
  public:
    static SlaveControlHandler &GetInstance()
    {
        static SlaveControlHandler instance;
        return instance;
    }
    std::unique_ptr<Message> ProcessMessage(const Message &message, SlaveDevice *device) override;
    SlaveControlHandler(const SlaveControlHandler &) = delete;
    SlaveControlHandler &operator=(const SlaveControlHandler &) = delete;

  private:
    SlaveControlHandler() = default;
};

} // namespace SlaveApp