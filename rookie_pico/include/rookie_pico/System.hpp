#ifndef SYSTEM_HPP
#define SYSTEM_HPP

// Pico headers
#include <pico/sync.h>

#include <hardware/watchdog.h>

#include <rookie_pico/Flag.hpp>

#include <pico_interface/PicoInterface.hpp>


static const uint32_t WATCHDOG_PERIOD_MS = 100;

// TODO [WF]: this feels unnecessary
enum class SYSTEM_STATE : uint8_t 
{
    STANDBY = static_cast<uint8_t>(pico_interface::Msg_SystemState::STATE::STANDBY),
    ESTOP = static_cast<uint8_t>(pico_interface::Msg_SystemState::STATE::ESTOP),
    ERROR = static_cast<uint8_t>(pico_interface::Msg_SystemState::STATE::ERROR),
    READY = static_cast<uint8_t>(pico_interface::Msg_SystemState::STATE::READY),
};

class System
{
public:

    System()
    {
        critical_section_init(&this->critical_section);
    };

    ~System()
    {
        this->FLAG._setState(Flag::STATE::STOP);
        critical_section_deinit(&this->critical_section);
    };

    void start()
    {
        watchdog_enable(WATCHDOG_PERIOD_MS, true);
    }

    void onCycle()
    {
        watchdog_update();

        // TODO: check the last time that we saw a heartbeat message...

    }

    SYSTEM_STATE getState()
    {
        SYSTEM_STATE returned;

        critical_section_enter_blocking(&this->critical_section);
        returned = this->state;
        critical_section_exit(&this->critical_section);

        return returned;
    };

    // TODO: might be redundant; why do this here when I could just 
    // do it inside the private version, _setState()?
    bool setState([[maybe_unused]] SYSTEM_STATE new_state)
    {
        switch (this->state) {
            case SYSTEM_STATE::STANDBY:
                return this->_stateStandby(new_state);
            case SYSTEM_STATE::ESTOP:
                return this->_stateEstop(new_state);
            case SYSTEM_STATE::ERROR:
                [[fallthrough]];
            case SYSTEM_STATE::READY:
                [[fallthrough]];
            default:
                break;
        };
        
        return true;
    };

    bool handleCommand(const std::string command)
    {
        pico_interface::Msg_SystemState state_cmd;
        pico_interface::message_error_t result = pico_interface::unpack_SystemState(
            command, state_cmd);
        if (result != pico_interface::E_MSG_SUCCESS) {
            this->status = pico_interface::MESSAGE_GET_ERROR(result);
            return false;
        }

        return setState(static_cast<SYSTEM_STATE>(state_cmd.state));
    };

    bool handleHeartbeat(const std::string heartbeat)
    {
        pico_interface::Msg_Heartbeat hbt;
        pico_interface::message_error_t result = pico_interface::unpack_Heartbeat(
            heartbeat, hbt);
        if (result != pico_interface::E_MSG_SUCCESS) 
        {
            this->status = pico_interface::MESSAGE_GET_ERROR(result);
            return false;
        }

        if (this->last_heartbeat_seq >= hbt.seq) 
        {
            // Sender either dropped out or reset since the last time we checked...
            this->setState(SYSTEM_STATE::STANDBY);
        }

        this->last_heartbeat_seq = hbt.seq;
        this->last_heartbeat_time = get_absolute_time();
    }

    void report()
    {
        pico_interface::Msg_SystemState state;
        state.state = static_cast<pico_interface::Msg_SystemState::STATE>(this->getState());
        state.status = this->status;

        std::string out;
        pico_interface::message_error_t result = pico_interface::pack_SystemState(
            state, pico_interface::MSG_ID_SYSTEM_STATE_STATUS, out);
        if (result != pico_interface::E_MSG_SUCCESS) {
            out = pico_interface::MESSAGE_GET_ERROR(result);
        }

        printf(out.c_str());
    };

    Flag* getFlag() { return &this->FLAG; };

protected:

private: // FUNCTIONS

    bool _stateStandby(SYSTEM_STATE new_state)
    {
        switch (new_state)
        {
            case SYSTEM_STATE::STANDBY:
                // NO-OP
                break;
            case SYSTEM_STATE::ESTOP:
                // No pre-conditions here. If someone requests an ESTOP, DO IT
                // this->FLAG._setState(Flag::STATE::STOP);
                this->_setState(SYSTEM_STATE::ESTOP, Flag::STATE::STOP);
                break;
            case SYSTEM_STATE::ERROR:
                // TODO: still don't know what the ERROR state will mean for this system
                break;
            case SYSTEM_STATE::READY:
                // TODO: what are the pre-conditions of READY?
                // What would need to be false here to cause a failure?
                // A failed POST?
                // For now, just allow it.
                this->_setState(SYSTEM_STATE::READY, Flag::STATE::OK);
                break;
            default:
                return false;
        }

        return true;
    };

    bool _stateEstop(SYSTEM_STATE new_state)
    {
        switch (new_state)
        {
            case SYSTEM_STATE::STANDBY:
                // There should probably be a set of conditions necessary for
                // this to happen, but eh...
                this->_setState(SYSTEM_STATE::STANDBY, Flag::STATE::STOP);
                break;
            case SYSTEM_STATE::ESTOP:
                // NO-OP
                break;
            case SYSTEM_STATE::ERROR:
                // Allow it, even though I don't know what it means.
                this->_setState(new_state, Flag::STATE::STOP);
                break;
            case SYSTEM_STATE::READY:
                // No. Need to go back through the STANDBY phase to 
                // get to READY from ESTOP.
                this->status = "Transition not allowed <ESTOP->READY>. Must go to STANDBY first.";
                return false;
            default:
                return false;
        }

        return true;
    };

    void _setState(SYSTEM_STATE new_state, Flag::STATE flag_state)
    {
        if (new_state == this->state) { return; }

        critical_section_enter_blocking(&this->critical_section);
        this->state = new_state;
        critical_section_exit(&this->critical_section);

        this->FLAG._setState(flag_state);
    };


private: // MEMBERS

    critical_section_t critical_section;

    SYSTEM_STATE state = SYSTEM_STATE::STANDBY;
    std::string status = "OK";

    // TODO: this should be handed to System as a ref/pointer, just like it
    // is to ClosedLoopController. Otherwise, we run the risk of null-ptr access
    // in the (hopefully unlikely) case that System goes down before ClosedLoopController.
    Flag FLAG;

    uint32_t last_heartbeat_seq;
    absolute_time_t last_heartbeat_time;

}; // class System


#endif // SYSTEM_HPP