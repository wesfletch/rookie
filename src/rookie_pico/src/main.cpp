#include <cstring>

#include <pico/stdlib.h>
#include <pico/stdio_usb.h>
#include <pico/multicore.h>
#include <hardware/watchdog.h>

#include <rookie_pico/ClosedLoopController.hpp>
#include <rookie_pico/Config.hpp>
#include <rookie_pico/Encoders.hpp>
#include <rookie_pico/main.h>
#include <rookie_pico/MessageHandler.hpp>
#include <rookie_pico/Motors.hpp>
#include <rookie_pico/OutboundQueue.hpp>
#include <rookie_pico/Pins.hpp>
#include <rookie_pico/System.hpp>

#include <pico_interface/PicoInterface.hpp>
#include <pico_interface/protos/Heartbeat.pb.hpp>

OutboundQueue outbound_queue;
queue_t inbound_queue;

bool
heartbeat_timer_callback([[maybe_unused]] struct repeating_timer* t)
{
    // Not handling possible overflows here... Not sure I care...
    heartbeat_seq += 1;

    Heartbeat hb = Heartbeat_init_zero;
    hb.seq = static_cast<int32_t>(heartbeat_seq);

    // We don't care if this fails or not, since a failure here indicates that the queue is full,
    // and the whole point of the heartbeat is to keep our connection when we're NOT sending.
    outbound_queue.try_enqueue(hb);

    // Returning true here ensures that the timer keeps firing.
    return true;
}

void
core_1_entry()
{
    enum class ReadState
    {
        WAIT_SYNC_0,
        WAIT_SYNC_1,
        READ_HEADER,
        READ_PAYLOAD
    };
    ReadState read_state = ReadState::WAIT_SYNC_0;

    pico_interface::Frame frame;
    size_t idx = 0;
    size_t payload_len = 0;

    while (true)
    {
        int ch = getchar_timeout_us(0);
        if (ch == PICO_ERROR_TIMEOUT)
        {
            // We've got a gap in inbound comms, now is a good time to drain
            // our outbound queue.
            outbound_queue.drain();
            continue;
        }

        uint8_t byte = static_cast<uint8_t>(ch);
        switch (read_state)
        {
            case ReadState::WAIT_SYNC_0:
                if (byte == pico_interface::SYNC_0)
                {
                    frame.buf[0] = byte;
                    read_state = ReadState::WAIT_SYNC_1;
                }
                break;
            case ReadState::WAIT_SYNC_1:
                if (byte == pico_interface::SYNC_1)
                {
                    frame.buf[1] = byte;
                    idx = 2;
                    read_state = ReadState::READ_HEADER;
                }
                else if (byte == pico_interface::SYNC_0)
                {
                    frame.buf[0] = byte;
                    // Stay in this state, we might have started reading the next frame early
                }
                else
                {
                    // Something went wrong. Fall back to the first state.
                    read_state = ReadState::WAIT_SYNC_0;
                }
                break;
            case ReadState::READ_HEADER:
                frame.buf[idx++] = byte;
                if (idx == pico_interface::FRAME_HEADER_SIZE)
                {
                    payload_len = static_cast<size_t>(frame.buf[2]) |
                                  (static_cast<size_t>(frame.buf[3]) << 8);
                    if (payload_len == 0)
                    {
                        frame.len = pico_interface::FRAME_HEADER_SIZE;
                        if (!queue_try_add(&inbound_queue, &frame))
                        {
                            pico_interface::Frame dropped;
                            queue_try_remove(&inbound_queue, &dropped);
                            queue_try_add(&inbound_queue, &frame);
                        }
                        read_state = ReadState::WAIT_SYNC_0;
                    }
                    else if (payload_len > pico_interface::MAX_PAYLOAD_SIZE)
                    {
                        // Oversized — discard and resync
                        read_state = ReadState::WAIT_SYNC_0;
                    }
                    else
                    {
                        read_state = ReadState::READ_PAYLOAD;
                    }
                }
                break;
            case ReadState::READ_PAYLOAD:
                frame.buf[idx++] = byte;
                if (idx == pico_interface::FRAME_HEADER_SIZE + payload_len)
                {
                    // We've successfully read a full frame. Push it across the queue
                    // to core 0 and restart from the top.
                    frame.len = idx;

                    // If our inbound queue is full, drop the oldest thing in the queue
                    // rather than ignoring potentially critical new information.
                    if (!queue_try_add(&inbound_queue, &frame))
                    {
                        pico_interface::Frame dropped;
                        queue_try_remove(&inbound_queue, &dropped);
                        queue_try_add(&inbound_queue, &frame);
                    }
                    read_state = ReadState::WAIT_SYNC_0;
                }
                break;
        }
    }
}

/**
 * @brief Program entrypoint.
 *
 * @return int
 */
int
main()
{
    stdio_init_all();
    stdio_set_translate_crlf(&stdio_usb, false);

    // Initialize the inbound/outbound queues that communicate between cores BEFORE
    // initializing the core1 routine
    queue_init(&inbound_queue, sizeof(pico_interface::Frame), (uint)10);
    outbound_queue.init();

    // Start core 1 - Do this before any interrupt configuration
    multicore_launch_core1(core_1_entry);

    // Configure the heartbeat timer.
    add_repeating_timer_ms(
        500 /** milliseconds */, heartbeat_timer_callback, nullptr, &heartbeat_timer);

    // Configure the blinking status light.
    struct repeating_timer status_led_timer;
    config_error_t led_status = configure_status_LED(&status_led_timer);
    if (led_status != E_CONFIG_SUCCESS)
    {
        printf("$ERR: failed to configure LED, returned CONFIG_ERROR=%d\n", led_status);
    }

    System system(&outbound_queue);

    // Configure our motor controller.
    std::shared_ptr<MDD10A> motor_controller = std::make_shared<MDD10A>(MDD10A(
        pins::motors::MDD10A_DIR_1_PIN,
        pins::motors::MDD10A_PWM_1_PIN,
        pins::motors::MDD10A_DIR_2_PIN,
        pins::motors::MDD10A_PWM_2_PIN));
    pwm_error_t motor_status = motor_controller->configure();
    if (motor_status != E_PWM_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    // Configure our encoders...
    std::shared_ptr<Encoder> leftEncoder = std::make_shared<Encoder>(
        Encoder(pins::encoders::LEFT_CHANNEL_A_GPIO, pins::encoders::LEFT_CHANNEL_B_GPIO));
    std::shared_ptr<Encoder> rightEncoder = std::make_shared<Encoder>(
        Encoder(pins::encoders::RIGHT_CHANNEL_A_GPIO, pins::encoders::RIGHT_CHANNEL_B_GPIO));
    EncoderList encoders = { leftEncoder, rightEncoder };
    init_encoders(&encoders);

    Flag controllerFlag;
    system.registerFlag(&controllerFlag, Flag::STATE::STOP);

    ClosedLoopController controller(
        motor_controller, leftEncoder, rightEncoder, &controllerFlag, &outbound_queue);

    IMessageHandler* handlers[] = { &system, &controller };

    // Responding to commands.
    pico_interface::Frame frame;
    pico_interface::message_error_t err;

    system.start();

    // spin
    while (1)
    {
        // Process any commands that might have been captured by core_1.
        if (queue_try_remove(&inbound_queue, &frame))
        {
            uint32_t msg_id;
            const uint8_t* payload;
            size_t payload_len;

            err = pico_interface::decode_frame(frame.buf, frame.len, msg_id, payload, payload_len);

            if (err != pico_interface::E_MSG_SUCCESS)
            {
                // TODO: enqueue Error proto
                continue;
            }

            for (auto* handler : handlers)
            {
                IMessageHandler::Result result = handler->handle(msg_id, payload, payload_len);
                if (result == IMessageHandler::Result::OK)
                {
                    // Handled successfully
                    break;
                }
                if (result == IMessageHandler::Result::ERROR)
                {
                    // TODO: enqueue Error proto
                    break;
                }
                // NOT_MINE -> continue to the next handler
            }
        }

        // ON_CYCLE
        system.onCycle();
        controller.onCycle();

        // REPORT
        system.report();
        controller.report();

        sleep_ms(20);
    }
}
