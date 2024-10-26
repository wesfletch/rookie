// CPP headers
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <cstring>

// Pico headers
#include "pico/stdlib.h"
#include "hardware/pwm.h"

// Project headers
#include <rookie_pico/Motors.hpp>
#include <pico_interface/PicoInterface.hpp>


pwm_error_t
MDD10A::configure()
{
    // configure pins for PWM
    gpio_set_function(this->pwm_1_pin, GPIO_FUNC_PWM);
    gpio_set_function(this->pwm_2_pin, GPIO_FUNC_PWM);

    // configure DIR pins as GPIO output
    gpio_set_function(this->dir_1_pin, GPIO_FUNC_SIO);
    gpio_set_function(this->dir_2_pin, GPIO_FUNC_SIO);
    gpio_set_dir(this->dir_1_pin, GPIO_OUT);
    gpio_set_dir(this->dir_2_pin, GPIO_OUT);

    // ensure that both pins are in the same "slice",
    // i.e., connected to the same PWM generator
    uint slice1 = pwm_gpio_to_slice_num(this->pwm_1_pin);
    uint slice2 = pwm_gpio_to_slice_num(this->pwm_2_pin);
    if (slice1 != slice2)
    {
        printf("ERROR: PWM slice mismatch, slice1: %d, slice2: %d", slice1, slice2);
        return E_PWM_SLICE_MISMATCH;
    }

    // set "wrap": number of cycles for each pulse
    pwm_set_wrap(slice1, PWM_WRAP);

    // start PWMs at 0 = STOP
    // each slice has two channels that we'll control separately
    // for this slice, assume that right is channel A, and left is channel B
    pwm_set_chan_level(slice1, PWM_CHAN_A, 0);     // right
    pwm_set_chan_level(slice1, PWM_CHAN_B, 0);     // left

    // set the PWM running
    pwm_set_enabled(slice1, true);

    return E_PWM_SUCCESS;
}

/**
 * @brief 
 * 
 * @param left_dir true for forward, false for reverse
 * @param left_speed [0-100] STOP --> FULL SPEED
 * @param right_dir true for forward, false for reverse
 * @param right_speed [0-100] STOP --> FULL SPEED
 */
pwm_error_t 
MDD10A::setMotors(
    bool dir_1, int speed_1, 
    bool dir_2, int speed_2)
{
    pwm_error_t status;

    // Set MOTOR_1
    status = this->setMotor(MOTOR::MOTOR_1, dir_1, speed_1);
    if (status != E_PWM_SUCCESS) {
        return status;
    }
    this->motor_1_pwm_command = speed_1;
    this->motor_1_dir_command = dir_1;

    // Set MOTOR_2
    status = this->setMotor(MOTOR::MOTOR_2, dir_2, speed_2);
    if (status != E_PWM_SUCCESS) {
        return status;
    }
    this->motor_2_pwm_command = speed_2;
    this->motor_2_dir_command = dir_2;

    return E_PWM_SUCCESS;
}

pwm_error_t
MDD10A::setMotor(
    MDD10A::MOTOR motor, bool dir, int speed)
{
    // Ensure duty-cycle percentage is in valid range
    if (speed != std::clamp(speed, 0, 100)) {
        return E_PWM_OUT_OF_RANGE;
    }

    switch (motor) {
    case MOTOR::MOTOR_1:
        gpio_put(this->dir_1_pin, dir);
        pwm_set_gpio_level(this->pwm_1_pin, getPwmValue(speed));
        break;
    case MOTOR::MOTOR_2:
        gpio_put(this->dir_2_pin, dir);
        pwm_set_gpio_level(this->pwm_2_pin, getPwmValue(speed));
        break;    
    }

    return E_PWM_SUCCESS;
}

bool
MDD10A::handleMsg_Motors(
    const std::string input)
{
    pico_interface::Msg_Motors motors = {};
    // attempt to unpack the message
    pico_interface::message_error_t unpacked = pico_interface::unpack_Motors(input, motors);
    if (unpacked != pico_interface::E_MSG_SUCCESS) 
    { 
        printf("$ERR: %s\n", MESSAGE_GET_ERROR(unpacked).c_str());
        return false;
    }


    pwm_error_t status = this->setMotors(
        (motors.motor_1_direction == pico_interface::Msg_Motors::DIRECTION::FORWARD) ? true : false,
        motors.motor_1_pwm,
        (motors.motor_2_direction == pico_interface::Msg_Motors::DIRECTION::FORWARD) ? true : false,
        motors.motor_2_pwm
    );
    if (status != E_PWM_SUCCESS) {
        printf("$ERR: Failed to set motor speeds.\n");
        return false;
    }

    return true;
}

void
MDD10A::report()
{
    std::string out;

    pico_interface::Msg_Motors commanded;
    commanded.motor_1_direction = static_cast<pico_interface::Msg_Motors::DIRECTION>(this->motor_1_dir_command);
    commanded.motor_1_pwm = this->motor_1_pwm_command;
    commanded.motor_2_direction = static_cast<pico_interface::Msg_Motors::DIRECTION>(this->motor_2_dir_command);
    commanded.motor_2_pwm = this->motor_2_pwm_command;

    pico_interface::message_error_t result = pico_interface::pack_Motors(
        commanded, pico_interface::MSG_ID_MOTORS_STATUS, out);
    if (result != pico_interface::E_MSG_SUCCESS)
    {
        out = pico_interface::MESSAGE_GET_ERROR(result);
    }

    printf(out.c_str());
}

