#include <string>
#include <cstring>

#include <rookie_pico/main.h>
#include <rookie_pico/Definitions.hpp>
#include <rookie_pico/Comms.hpp>
#include <rookie_pico/Motors.hpp>
#include <rookie_pico/Config.hpp>
#include <rookie_pico/Encoders.hpp>
#include <rookie_pico/Commands.hpp>
#include <rookie_pico/System.hpp>

#include <pico_interface/PicoInterface.hpp>


bool
heartbeat_timer_callback(
    [[maybe_unused]] struct repeating_timer* t)
{
    // Not handling possible overflows here... Not sure I care...
    heartbeat.seq += 1;

    std::string out;
    pico_interface::pack_HeartbeatMessage(heartbeat, out);

    printf(out.c_str());

    return true;
}

bool
handle_input(
    std::string input, 
    CommandCallbacks& callbacks)
{
    // split on string to get first 
    size_t posEndOfPrefix = input.find_first_of(" ");
    if (posEndOfPrefix == std::string::npos) {
        return false;
    }

    // split into prefix + the rest
    std::string prefix(input, 0, posEndOfPrefix);
    // printf("PS: %d, PREFIX: <%s>\n", (int)posEndOfPrefix, prefix.c_str());
    if (prefix.empty()) { return false; }

    std::string body = input.substr(posEndOfPrefix + 1);
    // printf("BODY: %s\n", body.c_str());

    // attempt to find this prefix in map
    auto it = callbacks.find(prefix);
    if (it == callbacks.end()) { return false; }

    bool result = it->second(body);
    return result;
}

/**
 * @brief Program entrypoint.
 * 
 * @return int 
 */
int main() 
{
    stdio_init_all();

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

    MotorControl controller(motor_controller, leftEncoder, rightEncoder);

    // std::shared_ptr<bool> motor_control_flag(std::move(controller.getFlag()));

    // Configure our "system state" handler. TBD how extensive this ends up being.
    std::vector<std::shared_ptr<bool>> flags = {
        // std::make_shared<bool>(std::move(controller.getFlag()))
        // motor_control_flag
        controller.getFlag()
    };

    System system(flags);

    // Set up the map of callbacks that will determine the message handlers for each inbound message type
    CommandCallbacks callbacks = {
        // This is the incantation necessary to make a callback to a member function
        // (test_callback), of an object (controller) with one parameter (placeholder)
        {pico_interface::MSG_ID_VELOCITY_CMD, std::bind(&MotorControl::handleCommand, controller, std::placeholders::_1)},
        {pico_interface::MSG_ID_MOTORS_CMD, std::bind(&MDD10A::handleMsg_Motors, motor_controller, std::placeholders::_1)},
        // {pico_interface::MSG_ID_SYSTEM_STATE_CMD, std::bind(&System::handleCommand, system, std::placeholders::_1)},
    };

    // STDIN/STDOUT IO
    char ch = ENDSTDIN;
    int idx = 0;
    char in_string[1024];
    bool success = false;

    // spin
    while (1)
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

                success = handle_input(in_string, callbacks);
                if (!success)
                {
                    printf("$ERR: Failed to process input: <%s>\n", in_string);
                }
                break;
            }
            // add latest char to string
            in_string[idx++] = ch;
            // get next char
            ch = static_cast<char>(getchar_timeout_us(0));
        }

        // controller.report();
        system.report();

        // // *(controller.getFlag()) = true;

        // controller.setVelocities(10, 10);
        controller.onCycle();
        // // motor_controller->setMotors(true, 75, true, 75);

        sleep_ms(20);
    }
}
