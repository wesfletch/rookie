#ifndef SYSTEM_HPP
#define SYSTEM_HPP

#include <vector>

// Pico headers
#include <pico/sync.h>
#include <hardware/watchdog.h>

#include <pico_interface/PicoInterface.hpp>
#include <pico_interface/protos/State.pb.hpp>
#include <pico_interface/protos/Heartbeat.pb.hpp>

#include <rookie_pico/Flag.hpp>
#include <rookie_pico/MessageHandler.hpp>
#include <rookie_pico/OutboundQueue.hpp>

static const uint32_t WATCHDOG_PERIOD_MS = 100;
static const uint32_t HEARTBEAT_TIMEOUT_PERIOD_MS = 500;

enum class SYSTEM_STATE : uint8_t
{
    // INIT = static_cast<uint8_t>(pico_interface::Msg_SystemState::STATE::INIT), // TODO
    STANDBY = static_cast<uint8_t>(State::State_STANDBY),
    ESTOP = static_cast<uint8_t>(State::State_ESTOP),
    ERROR = static_cast<uint8_t>(State::State_ERROR), // Could this maybe be "FAULT" instead?
    READY = static_cast<uint8_t>(State::State_READY),
    TEST = static_cast<uint8_t>(State::State_TEST),
};

class System : public IMessageHandler
{
public:

    System(OutboundQueue* outbound);

    // TODO: this doesn't work since we're trying to set flag with no guarantee that
    // any flags are registered.
    System(SYSTEM_STATE system_state, Flag::STATE flag_state, OutboundQueue* outbound);

    ~System();

    void start();

    void onCycle();

    SYSTEM_STATE getState();

    bool setState(SYSTEM_STATE new_state);

    Result handle(uint32_t msg_id, const uint8_t* payload, std::size_t len) override;

    bool handleCommand(const StateCommand& cmd);

    bool handleHeartbeat(const Heartbeat& heartbeat);

    void report();

    // Flag* getFlag() { return &this->FLAG; };

    void registerFlag(Flag* flag, Flag::STATE flag_state = Flag::STATE::STOP);

    void reset();

protected:
private: // FUNCTIONS

    bool _stateStandby(SYSTEM_STATE new_state);

    bool _stateEstop(SYSTEM_STATE new_state);

    bool _stateError(SYSTEM_STATE new_state);

    bool _stateReady(SYSTEM_STATE new_state);

    bool _stateTest(SYSTEM_STATE new_state);

    void _setState(SYSTEM_STATE new_state, Flag::STATE flag_state);

    void _setFlags(Flag::STATE flag_state);

    critical_section_t critical_section;

    SYSTEM_STATE state = SYSTEM_STATE::STANDBY;
    std::string status = "OK";

    OutboundQueue* _outbound;

    // TODO: this should be handed to System as a ref/pointer, just like it
    // is to ClosedLoopController. Otherwise, we run the risk of null-ptr access
    // in the (hopefully unlikely) case that System goes down before ClosedLoopController.
    std::vector<Flag*> flags;
    // Flag FLAG;

    uint32_t last_heartbeat_seq = 0;
    absolute_time_t last_heartbeat_time = nil_time;

}; // class System

#endif // SYSTEM_HPP
