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
#include <rookie_pico/PicoInterface.hpp>


pwm_error_t
MDD10A::configure()
{
    // configure pins for PWM
    gpio_set_function(pwm_1_pin, GPIO_FUNC_PWM);
    gpio_set_function(pwm_2_pin, GPIO_FUNC_PWM);

    // configure DIR pins as GPIO output
    gpio_set_function(dir_1_pin, GPIO_FUNC_SIO);
    gpio_set_function(dir_2_pin, GPIO_FUNC_SIO);
    gpio_set_dir(dir_1_pin, GPIO_OUT);
    gpio_set_dir(dir_2_pin, GPIO_OUT);

    // ensure that both pins are in the same "slice",
    // i.e., connected to the same PWM generator
    uint slice1 = pwm_gpio_to_slice_num(pwm_1_pin);
    uint slice2 = pwm_gpio_to_slice_num(pwm_2_pin);
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

    // Set MOTOR_2
    status = this->setMotor(MOTOR::MOTOR_2, dir_2, speed_2);
    if (status != E_PWM_SUCCESS) {
        return status;
    }

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

/**
 * @brief Set the speed of a single motor
 * 
 * @param pwm_pin 
 * @param dir_pin 
 * @param speed 
 * @param dir 
 * @return pwm_error_t 
 */
pwm_error_t
set_motor_MDD10A(
    uint pwm_pin, uint dir_pin,
    uint8_t speed, bool dir)
{
    // set DIR pin
    gpio_put(dir_pin, dir);
    // set PWM pin
    pwm_set_gpio_level(pwm_pin, getPwmValue(speed));

    return E_PWM_SUCCESS;
}

bool
MDD10A::handleMsg_Motors(
    const std::string input)
{
    Msg_MotorsCommand motors = {};
    // attempt to unpack the message
    message_error_t unpacked = unpack_MotorsCommand(input, motors);
    if (unpacked != E_MSG_SUCCESS) 
    { 
        printf("$ERR: %s\n", MESSAGE_GET_ERROR(unpacked).c_str());
        return false;
    }

    // Read-back for debugging
    std::string out;
    message_error_t packed = pack_MotorsCommand(motors, out);
    if (packed != E_MSG_SUCCESS) 
    {
        printf("$ERR: %s\n", MESSAGE_GET_ERROR(packed).c_str());
        return false;
    }
    printf("OUT: <%s>\n", out.c_str());


    pwm_error_t status = this->setMotors(
        (motors.motor_1_direction == Msg_MotorsCommand::DIRECTION::FORWARD) ? true : false,
        motors.motor_1_pwm,
        (motors.motor_2_direction == Msg_MotorsCommand::DIRECTION::FORWARD) ? true : false,
        motors.motor_2_pwm
    );
    if (status != E_PWM_SUCCESS) {
        printf("Fuck");
        return false;
    }

    return true;
}


MotorControl::MotorControl(
    std::shared_ptr<MDD10A> controller,
    std::shared_ptr<Encoder> encoder1,
    std::shared_ptr<Encoder> encoder2)
{
    this->controller = controller;
    this->encoder1 = encoder1;
    this->encoder2 = encoder2;

    // initialize our mutexes
    mutex_init(&this->desired_vel_mtx);
    mutex_init(&this->current_vel_mtx);

    // initialize the watchdog timer to zero out vels if
    // we don't receive commands fast enough

}

// bool
// MotorControl::init()
// {
//     // configure MDD10A motor controller
//     // realistically, there's no reason for all of the complexity
//     // about which motor controller we're using, so let's just KISS
//     pwm_error_t pwm_status = configure_MDD10A();
//     if (pwm_status)
//     {
//         printf("$ERR: Failed to configure PWM.\n");
//         return false;
//     }

//     return true;
// }

void
MotorControl::setVelocity(
    float desired_velocity)
{
    mutex_enter_blocking(&this->desired_vel_mtx);
    
    // TODO: should probably do some checks here, but...
    this->desired_velocity = desired_velocity;
    
    mutex_exit(&this->desired_vel_mtx);
}

float
MotorControl::getVelocity()
{
    float returned = 0.0;

    mutex_enter_blocking(&this->current_vel_mtx);
    
    // TODO: should probably do some checks here, but...
    returned = this->current_velocity;
    
    mutex_exit(&this->current_vel_mtx);

    return returned;
}

// bool
// MotorControl::onCycle()
// {

//     // map desired_velocity (in rads/sec) to a pwm duty-cycle [0,100]

//     uint16_t pwm_value = getPwmValue(getPwmDutyCycle(this->desired_velocity));

//     set_motor_MDD10A(
//         this->pwm_pin, this->dir_pin,
//         desired_velocity, 
//         (desired_velocity >= 0) ? true : false
//     );
// }

