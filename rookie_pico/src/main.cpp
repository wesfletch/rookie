#include <string>
#include <cstring>

#include <pico/multicore.h>

#include <rookie_pico/main.h>
#include <rookie_pico/Motors.hpp>
#include <rookie_pico/ClosedLoopController.hpp>
#include <rookie_pico/Config.hpp>
#include <rookie_pico/Encoders.hpp>
#include <rookie_pico/Commands.hpp>
#include <rookie_pico/System.hpp>

#include <pico_interface/PicoInterface.hpp>

// for string parsing
#define ENDSTDIN    255 // NULL
#define NL          10  // NEWLINE
#define CR          13  // CARRIAGE RETURN


bool
heartbeat_timer_callback(
    [[maybe_unused]] struct repeating_timer* t)
{
    // Not handling possible overflows here... Not sure I care...
    heartbeat.seq += 1;

    std::string out;
    pico_interface::pack_HeartbeatMessage(heartbeat, out);

    printf(out.c_str());

    // Returning true here ensures that the timer keeps firing.
    return true;
}

bool
split_input(
    std::string input, 
    const CommandCallbacks& callbacks,
    std::string& header,
    std::string& body)
{
    printf("RAW: <%s>\n", input.c_str());
    // split on string to get first 
    size_t posEndOfPrefix = input.find_first_of(" ");
    if (posEndOfPrefix == std::string::npos)
    {
        return false;
    }

    // split into prefix + the rest
    std::string prefix(input, 0, posEndOfPrefix);
    if (prefix.empty()) 
    { 
        printf("$ERR: Failed to split prefix from body.\n");
        return false; 
    }

    header = prefix;
    body = input.substr(posEndOfPrefix + 1);

    return true;
}

pico_interface::Msg_Ack
create_ack(
    std::string header, 
    std::string body, 
    const bool status)
{
    pico_interface::Msg_Ack ack;

    ack.header = header;
    ack.fields = body;
    ack.status = (status) ? 
        pico_interface::Msg_Ack::STATUS::SUCCESS :
        pico_interface::Msg_Ack::STATUS::FAILURE;

    return ack;
}

bool
call_command(
    const std::string header, 
    const std::string body,
    const CommandCallbacks& callbacks)
{
    auto it = callbacks.find(header);
    if (it == callbacks.end()) 
    {
        printf("$ERR: Failed to match prefix <%s> to callback.\n", prefix.c_str());
        return false; 
    }

    // Call the command associated with the header.
    bool status = it->second(body);
    return status;
}

queue_t intercore_queue;

void
core_1_entry()
{
    // STDIN/STDOUT IO
    char ch = ENDSTDIN;
    int idx = 0;
    char in_string[1024];

    while (true) 
    {
        // attempt to read char from stdin, non-blocking
        ch = static_cast<char>(getchar_timeout_us(0));
        while (ch != ENDSTDIN)
        {
            // if the string ends or we run out of space, we're done with this string
            if (ch == CR || ch == NL || idx == (sizeof(in_string)-1))
            {
                in_string[idx] = '\0'; // null-terminate the string
                idx = 0;    // reset index

                queue_add_blocking(&intercore_queue, &in_string);

                break;
            }
            // add latest char to string
            in_string[idx++] = ch;
            // get next char
            ch = static_cast<char>(getchar_timeout_us(0));
        }
        ch = ENDSTDIN;
    }
}

/**
 * @brief Program entrypoint.
 * 
 * @return int 
 */
int main() 
{
    stdio_init_all();

    queue_init(&intercore_queue, (uint)(1024 * sizeof(char)), (uint)10);

    multicore_launch_core1(core_1_entry); // Start core 1 - Do this before any interrupt configuration

    // Configure the heartbeat timer.
    add_repeating_timer_ms(
        500 /** milliseconds */,
        heartbeat_timer_callback,
        NULL,
        &heartbeat_timer);

    // Configure the blinking status light.
    struct repeating_timer status_led_timer;
    config_error_t led_status = configure_status_LED(&status_led_timer);
    if (led_status != E_CONFIG_SUCCESS)
    {
        printf("$ERR: failed to configure LED, returned CONFIG_ERROR=%d\n", led_status);
    }

    System system;
    
    // Configure our motor controller.
    std::shared_ptr<MDD10A> motor_controller = std::make_shared<MDD10A>(
        MDD10A(MDD10A_DIR_1_PIN, MDD10A_PWM_1_PIN,
               MDD10A_DIR_2_PIN, MDD10A_PWM_2_PIN));
    pwm_error_t motor_status = motor_controller->configure();
    if (motor_status != E_PWM_SUCCESS) {
        return EXIT_FAILURE;
    }

    // Configure our encoders...
    std::shared_ptr<Encoder> leftEncoder = std::make_shared<Encoder>(
        Encoder(LEFT_CHANNEL_A_GPIO, LEFT_CHANNEL_B_GPIO));
    std::shared_ptr<Encoder> rightEncoder = std::make_shared<Encoder>(
        Encoder(RIGHT_CHANNEL_A_GPIO, RIGHT_CHANNEL_B_GPIO));

    EncoderList encoders = {leftEncoder, rightEncoder};
    struct repeating_timer encoder_timer;
    init_encoders(&encoders, &encoder_timer);

    ClosedLoopController controller(
        motor_controller, 
        leftEncoder, 
        rightEncoder,
        system.getFlag());

    // Set up the map of callbacks that will determine the message handlers for each inbound message type
    const CommandCallbacks callbacks = {
        // This is the incantation necessary to make a callback to a member function
        // (test_callback), of a POINTER TO AN OBJECT (controller) with one parameter (placeholder)
        {pico_interface::MSG_ID_VELOCITY_CMD, std::bind(&ClosedLoopController::handleCommand, controller, std::placeholders::_1)},
        {pico_interface::MSG_ID_MOTORS_CMD, std::bind(&MDD10A::handleMsg_Motors, motor_controller, std::placeholders::_1)},
        {pico_interface::MSG_ID_SYSTEM_STATE_CMD, std::bind(&System::handleCommand, &system, std::placeholders::_1)},
        {pico_interface::MSG_ID_HEARTBEAT, std::bind(&System::handleHeartbeat, &system, std::placeholders::_1)}
    };

    // STDIN/STDOUT IO
    char input[1024];
    bool success = false;

    // Responding to commands.
    pico_interface::Msg_Ack ack;
    pico_interface::message_error_t result;


    // spin
    while (1)
    {
        // Process any commands that might have been captured by core_1.
        if (queue_try_remove(&intercore_queue, &input)) 
        {
            std::string header;
            std::string body;
            if (!split_input(std::string(input), callbacks, header, body)) 
            {
                printf("$ERR: Failed to process input: <%s>\n", input);
                continue;
            }

            success = call_command(header, body, callbacks);
            ack = create_ack(header, body, success);

            std::string ack_string;
            result = pico_interface::pack_Ack(ack, ack_string);
            if (result != pico_interface::E_MSG_SUCCESS) 
            {
                ack_string = pico_interface::MESSAGE_GET_ERROR(result);
            }
            printf(ack_string.c_str());
        }

        static bool started = false;
        if (!started) {
            system.start();
            started = true;
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
