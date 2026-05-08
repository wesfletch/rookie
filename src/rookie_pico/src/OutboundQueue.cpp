#include <cstdio>

#include <pico_interface/PicoInterface.hpp>

#include <rookie_pico/Config.hpp>
#include <rookie_pico/OutboundQueue.hpp>

static void
write_frame_stdio(const pico_interface::Frame& frame)
{
    fwrite(frame.buf, 1, frame.len, stdout);
    fflush(stdout);
}

static void
write_frame_uart(const pico_interface::Frame& frame)
{
    uart_write_blocking(IO_UART_ID, frame.buf, frame.len);
}

void
OutboundQueue::init()
{
    if (this->_init)
    {
        return;
    }
    queue_init(&_queue, sizeof(pico_interface::Frame), MAX_QUEUE_DEPTH);
    this->_init = true;
}

void
OutboundQueue::drain()
{
    hard_assert(_init, "Attempt to use OutboundQueue before init().");

    pico_interface::Frame frame;
    while (queue_try_remove(&_queue, &frame))
    {
#ifdef OUTBOUND_USE_UART
        write_frame_uart(frame);
#else
        write_frame_stdio(frame);
#endif
    }
}