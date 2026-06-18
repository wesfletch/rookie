#ifndef CLOCK_HPP
#define CLOCK_HPP

#include <PicoInterface.hpp>
#include <pb.h>
#include <pico/stdlib.h>
#include <pico/time.h>
#include <pico/types.h>

#include <pico_interface/PicoInterface.hpp>
#include <pico_interface/protos/Clock.pb.hpp>

#include <rookie_pico/MessageHandler.hpp>
#include <rookie_pico/OutboundQueue.hpp>

namespace rookie::clock
{

class ITimeSource
{
public:

    virtual ~ITimeSource() = default;
    virtual uint64_t now_us() const = 0;
}; // ITimeSource

class SystemTimeSource final : public ITimeSource
{
public:

    uint64_t
    now_us() const override
    {
        return to_us_since_boot(get_absolute_time());
    }
};

class Clock : public IMessageHandler
{
public:

    Clock(ITimeSource& source, OutboundQueue* outbound) : _source(source), _outbound(outbound){};

    uint64_t
    now() const
    {
        return _source.now_us();
    }

    IMessageHandler::Result
    handle([[maybe_unused]] uint32_t msg_id, const uint8_t* payload, std::size_t len) override
    {
        pico_interface::message_error_t err;
        switch (msg_id)
        {
            case nanopb::MessageDescriptor<ClockRequest>::msgid():
                ClockRequest req;
                err = pico_interface::decode_payload(req, payload, len);
                if (err != pico_interface::E_MSG_SUCCESS)
                {
                    return IMessageHandler::Result::ERROR;
                }

                if (!this->handleClockRequest(req))
                {
                    return IMessageHandler::Result::ERROR;
                }
                break;
            default:
                return IMessageHandler::Result::NOT_MINE;
        }

        return IMessageHandler::Result::OK;
    };

    bool
    handleClockRequest(const ClockRequest& req)
    {
        ClockReport clk = ClockReport_init_zero;
        clk.seq = req.seq;
        clk.clock_us = this->now();
        return this->_outbound->try_enqueue(clk);
    }

private:

    ITimeSource& _source;

    OutboundQueue* _outbound;
}; // class Clock

} // namespace rookie::clock

#endif // CLOCK_HPP