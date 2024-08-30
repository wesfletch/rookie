#ifndef SYSTEM_HPP
#define SYSTEM_HPP

// Pico headers
#include <pico/sync.h>

#include <rookie_pico/Flag.hpp>

#include <pico_interface/PicoInterface.hpp>


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

    System(std::vector<std::shared_ptr<bool>> flags)
    {
        critical_section_init(&this->critical_section);

        this->flags = flags;
        this->_markFlags(true);
    };

    ~System()
    {
        this->_markFlags(false);
        critical_section_deinit(&this->critical_section);
    };

    SYSTEM_STATE getState()
    {
        SYSTEM_STATE returned;

        critical_section_enter_blocking(&this->critical_section);
        returned = this->state;
        critical_section_exit(&this->critical_section);

        return returned;
    };

    bool setState([[maybe_unused]] SYSTEM_STATE new_state)
    {
        switch (this->state) {
            case SYSTEM_STATE::STANDBY:
                [[fallthrough]];
            case SYSTEM_STATE::ESTOP:
                [[fallthrough]];
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
        }

        return setState(static_cast<SYSTEM_STATE>(state_cmd.state));
    };

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

protected:

private: // FUNCTIONS

    void _setState(SYSTEM_STATE new_state)
    {
        if (new_state == this->state) { return; }

        critical_section_enter_blocking(&this->critical_section);
        this->state = new_state;
        critical_section_exit(&this->critical_section);
    };

    void _markFlags(const bool enabled)
    {
        for (auto& flag : this->flags)
        {
            *flag = enabled;
        }
    };

private: // MEMBERS

    critical_section_t critical_section;

    SYSTEM_STATE state = SYSTEM_STATE::STANDBY;
    std::string status = "";

    std::vector<std::shared_ptr<bool>> flags = {}; 

}; // class System


#endif // SYSTEM_HPP