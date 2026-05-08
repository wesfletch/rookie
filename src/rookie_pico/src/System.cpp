#include <cstring>

#include <pico.h>

#include <pico_interface/PicoInterface.hpp>
#include <pico_interface/protos/State.pb.hpp>
#include <pico_interface/protos/Reset.pb.hpp>

#include <rookie_pico/MessageHandler.hpp>
#include <rookie_pico/System.hpp>

System::System(OutboundQueue* outbound) : _outbound(outbound)
{
    critical_section_init(&this->critical_section);

    // Reinit outbound queue here, just in case.
    this->_outbound->init();
}

// TODO: this doesn't work since we're trying to set flag with no guarantee that
// any flags are registered.
System::System(SYSTEM_STATE system_state, Flag::STATE flag_state, OutboundQueue* outbound)
    : System(outbound)
{
    this->_setState(system_state, flag_state);
}

System::~System()
{
    // this->FLAG._setState(Flag::STATE::STOP);
    this->_setFlags(Flag::STATE::STOP);
    critical_section_deinit(&this->critical_section);
}

void
System::start()
{
    watchdog_enable(WATCHDOG_PERIOD_MS, true);
}

void
System::onCycle()
{
    if (this->state == SYSTEM_STATE::TEST)
    {
        // disable the watchdog
        hw_clear_bits(&watchdog_hw->ctrl, WATCHDOG_CTRL_ENABLE_BITS);
    }
    else
    {
        watchdog_update();
    }

    // TODO: check the last time that we saw a heartbeat message...
    absolute_time_t timeout = delayed_by_ms(this->last_heartbeat_time, HEARTBEAT_TIMEOUT_PERIOD_MS);
    if (absolute_time_diff_us(timeout, get_absolute_time()) < 0)
    {
        // We've gone too long without a heartbeat...
        // Reset ourselves to STANDBY, and clear any heartbeat info we have.
        this->setState(SYSTEM_STATE::STANDBY);
        this->last_heartbeat_seq = 0;
        this->last_heartbeat_time = nil_time;
    }
}

SYSTEM_STATE
System::getState()
{
    SYSTEM_STATE returned;

    critical_section_enter_blocking(&this->critical_section);
    returned = this->state;
    critical_section_exit(&this->critical_section);

    return returned;
}

bool
System::setState(SYSTEM_STATE new_state)
{
    switch (this->state)
    {
        case SYSTEM_STATE::STANDBY:
            return this->_stateStandby(new_state);
        case SYSTEM_STATE::ESTOP:
            return this->_stateEstop(new_state);
        case SYSTEM_STATE::ERROR:
            return this->_stateError(new_state);
        case SYSTEM_STATE::READY:
            return this->_stateReady(new_state);
        case SYSTEM_STATE::TEST:
            return this->_stateTest(new_state);
        default:
            return false;
    };

    return true;
}

IMessageHandler::Result
System::handle(uint32_t msg_id, const uint8_t* payload, std::size_t len)
{
    pico_interface::message_error_t err;

    switch (msg_id)
    {
        case nanopb::MessageDescriptor<StateCommand>::msgid():
            StateCommand cmd;
            err = pico_interface::decode_payload(cmd, payload, len);
            if (err != pico_interface::E_MSG_SUCCESS)
            {
                return IMessageHandler::Result::ERROR;
            }

            if (!setState(static_cast<SYSTEM_STATE>(cmd.command_state)))
            {
                return IMessageHandler::Result::ERROR;
            }
            return IMessageHandler::Result::OK;
        case nanopb::MessageDescriptor<Reset>::msgid():
            this->reset();
            panic("THIS SHOULDN'T HAPPEN, RESET SHOULD NEVER RETURN!!!!");
        default:
            return IMessageHandler::Result::NOT_MINE;
    }

    return IMessageHandler::Result::OK;
}

bool
System::handleCommand(const StateCommand& cmd)
{
    if (!setState(static_cast<SYSTEM_STATE>(cmd.command_state)))
    {
        return false;
    }
    this->status = "OK";
    return true;
}

bool
System::handleHeartbeat(const Heartbeat& heartbeat)
{
    if (this->last_heartbeat_seq >= heartbeat.seq)
    {
        // Sender either dropped out or reset since the last time we checked...
        // We'll return to STANDBY.
        this->setState(SYSTEM_STATE::STANDBY);
    }

    this->last_heartbeat_seq = heartbeat.seq;
    this->last_heartbeat_time = get_absolute_time();

    return true;
}

void
System::reset()
{
    // Enable the watchdog that will reset the MCU after 1sec...
    watchdog_enable(1000, 1);

    // ... and then block indefinitely until we reset.
    while (1)
        ;
}

void
System::report()
{
    StateReport report = StateReport_init_zero;
    report.current_state = static_cast<State>(this->getState());
    report.has_status = true;
    strncpy(report.status, this->status.c_str(), sizeof(report.status) - 1);

    this->_outbound->try_enqueue(report);
}

void
System::registerFlag(Flag* flag, Flag::STATE flag_state)
{
    flag->_setState(flag_state);
    this->flags.push_back(flag);
}

bool
System::_stateStandby(SYSTEM_STATE new_state)
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
            this->_setState(SYSTEM_STATE::ERROR, Flag::STATE::STOP);
            break;
        case SYSTEM_STATE::READY:
            // TODO: what are the pre-conditions of READY?
            // What would need to be false here to cause a failure?
            // A failed POST?
            // For now, just allow it.
            this->_setState(SYSTEM_STATE::READY, Flag::STATE::OK);
            break;
        case SYSTEM_STATE::TEST:
            this->_setState(SYSTEM_STATE::TEST, Flag::STATE::OK);
            break;
        default:
            return false;
    }

    return true;
}

bool
System::_stateEstop(SYSTEM_STATE new_state)
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
        case SYSTEM_STATE::TEST:
            this->status = "Transition not allowed <ESTOP->TEST>. Must go to STANDBY first.";
            return false;
        default:
            return false;
    }

    return true;
}

bool
System::_stateError(SYSTEM_STATE new_state)
{
    switch (new_state)
    {
        case SYSTEM_STATE::STANDBY:
            // TODO: There should probably be a set of conditions necessary for
            // this to happen, but eh...
            this->_setState(SYSTEM_STATE::STANDBY, Flag::STATE::STOP);
            break;
        case SYSTEM_STATE::ESTOP:
            [[fallthrough]];
            // This is meaningless. ERROR enforces same constraints as ESTOP,
            // for now. In the future, it might be worthwhile to separate these...
            // NO-OP
        case SYSTEM_STATE::ERROR:
            // NO-OP
            break;
        case SYSTEM_STATE::READY:
            // No. Need to go back through the STANDBY phase to
            // get to READY from ERROR.
            this->status = "Transition not allowed <ERROR->READY>. Must go to STANDBY first.";
            return false;
        case SYSTEM_STATE::TEST:
            this->status = "Transition not allowed <ERROR->TEST>. Must go to STANDBY first.";
            return false;
        default:
            return false;
    }

    return true;
}

bool
System::_stateReady(SYSTEM_STATE new_state)
{
    switch (new_state)
    {
        case SYSTEM_STATE::STANDBY:
            this->_setState(SYSTEM_STATE::STANDBY, Flag::STATE::STOP);
            break;
        case SYSTEM_STATE::ESTOP:
            this->_setState(SYSTEM_STATE::ESTOP, Flag::STATE::OK);
            break;
        case SYSTEM_STATE::ERROR:
            // Allow it, even though I don't know what it means.
            this->_setState(new_state, Flag::STATE::STOP);
            break;
        case SYSTEM_STATE::READY:
            // NO-OP
            break;
        case SYSTEM_STATE::TEST:
            this->status = "Transition not allowed <READY->TEST>. Must go to STANDBY first.";
            return false;
        default:
            return false;
    }

    return true;
}

bool
System::_stateTest(SYSTEM_STATE new_state)
{
    switch (new_state)
    {
        case SYSTEM_STATE::STANDBY:
            // re-enable the watchdog timer
            watchdog_enable(WATCHDOG_PERIOD_MS, true);
            this->_setState(SYSTEM_STATE::STANDBY, Flag::STATE::STOP);
            break;
        case SYSTEM_STATE::ESTOP:
            [[fallthrough]];
        case SYSTEM_STATE::ERROR:
            [[fallthrough]];
        case SYSTEM_STATE::READY:
            this->status = "Transition from TEST not allowed. Only valid transition from TEST "
                           "is to STANDBY.";
            return false;
        case SYSTEM_STATE::TEST:
            // no-op
            break;
        default:
            return false;
    }

    return true;
}

void
System::_setState(SYSTEM_STATE new_state, Flag::STATE flag_state)
{
    if (new_state == this->state)
    {
        return;
    }

    critical_section_enter_blocking(&this->critical_section);
    this->state = new_state;
    critical_section_exit(&this->critical_section);

    // this->FLAG._setState(flag_state);
    this->_setFlags(flag_state);
}

void
System::_setFlags(Flag::STATE flag_state)
{
    for (auto const& flag : this->flags)
    {
        flag->_setState(flag_state);
    }
}
