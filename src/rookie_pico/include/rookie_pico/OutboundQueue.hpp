#ifndef ROOKIE_PICO_OUTBOUND_QUEUE_HPP
#define ROOKIE_PICO_OUTBOUND_QUEUE_HPP

#include <pico/assert.h>
#include <pico/util/queue.h>

#include <pico_interface/PicoInterface.hpp>

class OutboundQueue
{
public:

    static constexpr size_t MAX_QUEUE_DEPTH = 10;

    void init();

    template <typename T>
    bool
    try_enqueue(const T& msg)
    {
        hard_assert(_init, "Attempt to use OutboundQueue before init().");

        pico_interface::Frame frame;
        pico_interface::message_error_t result =
            pico_interface::encode_frame(msg, frame.buf, pico_interface::MAX_FRAME_SIZE, frame.len);

        // Encoding failures are never a valid runtime condition
        hard_assert(
            result == pico_interface::E_MSG_SUCCESS,
            "encode_frame failed: %d:'%s'",
            result,
            pico_interface::MESSAGE_GET_ERROR(result).c_str());

        return queue_try_add(&_queue, &frame);
    }

    template <typename T>
    void
    enqueue(const T& msg)
    {
        hard_assert(_init, "Attempt to use OutboundQueue before init().");

        pico_interface::Frame frame;
        pico_interface::message_error_t result =
            pico_interface::encode_frame(msg, frame.buf, pico_interface::MAX_FRAME_SIZE, frame.len);

        // Encoding failures are never a valid runtime condition
        hard_assert(
            result == pico_interface::E_MSG_SUCCESS,
            "encode_frame failed: %d:'%s'",
            result,
            pico_interface::MESSAGE_GET_ERROR(result).c_str());

        queue_add_blocking(&_queue, &frame);
    }

    void drain();

protected:
private:

    bool _init = false;

    queue_t _queue;

}; // class OutboundQueue

#endif // ROOKIE_PICO_OUTBOUND_QUEUE_HPP