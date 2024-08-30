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

bool
MDD10A::handleMsg_Motors(
    const std::string input)
{
    pico_interface::Msg_MotorsCommand motors = {};
    // attempt to unpack the message
    pico_interface::message_error_t unpacked = pico_interface::unpack_MotorsCommand(input, motors);
    if (unpacked != pico_interface::E_MSG_SUCCESS) 
    { 
        printf("$ERR: %s\n", MESSAGE_GET_ERROR(unpacked).c_str());
        return false;
    }

    // Read-back for debugging
    std::string out;
    pico_interface::message_error_t packed = pico_interface::pack_MotorsCommand(motors, out);
    if (packed != pico_interface::E_MSG_SUCCESS) 
    {
        printf("$ERR: %s\n", MESSAGE_GET_ERROR(packed).c_str());
        return false;
    }
    printf("OUT: <%s>\n", out.c_str());


    pwm_error_t status = this->setMotors(
        (motors.motor_1_direction == pico_interface::Msg_MotorsCommand::DIRECTION::FORWARD) ? true : false,
        motors.motor_1_pwm,
        (motors.motor_2_direction == pico_interface::Msg_MotorsCommand::DIRECTION::FORWARD) ? true : false,
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
    mutex_init(&this->motor_1.mtx);
    mutex_init(&this->motor_2.mtx);

    this->FLAG = std::make_shared<Flag>();
}

bool
MotorControl::onCycle()
{
    if (!this->FLAG) 
    {
        this->setVelocities(0.0, 0.0);
        this->status = "Flag disabled. Velocities zeroed.\n";
        printf(status.c_str());
    }

    // // Timeout condition
    // if (absolute_time_diff_us(delayed_by_ms(this->last_cmd_time, 1000), get_absolute_time()) < 0)
    // {
    //     this->setVelocities(0.0, 0.0);
    //     this->status = "Velocity command time-out. Velocities zeroed.";
    // }

    // Get encoder speeds

    // Actually assert our motor velocities here
    // TODO: need to convert these to PWMs
    std::tuple<float, float> desired_vels = this->getDesiredVelocities();

    // velocities that are handed to us are in rads/s
    float motor_1_speed_rads = std::get<0>(desired_vels);
    float motor_2_speed_rads = std::get<1>(desired_vels);

    // get the signs
    bool motor_1_dir = (motor_1_speed_rads > 0) ? true : false;
    bool motor_2_dir = (motor_2_speed_rads > 0) ? true : false;

    // convert radians/sec to RPM and dump the signs
    float motor_1_rpm = std::abs((motor_1_speed_rads * 60) / (2 * M_PI));
    float motor_2_rpm = std::abs((motor_2_speed_rads * 60) / (2 * M_PI));

    // get the PWM value that would be necessary to reach this speed
    int motor_1_pwm = (motor_1_rpm / MOTOR_MAX_RPM) * 100;
    int motor_2_pwm = (motor_2_rpm / MOTOR_MAX_RPM) * 100;

    // clamp to our valid pwm range
    motor_1_pwm = std::clamp(motor_1_pwm, 0, 100);
    motor_2_pwm = std::clamp(motor_2_pwm, 0, 100);

    this->controller->setMotors(
        motor_1_dir, motor_1_pwm,
        motor_2_dir, motor_2_pwm
    );

    return true;
}

bool
MotorControl::handleCommand(
    const std::string command)
{
    pico_interface::Msg_Velocity vel;
    pico_interface::message_error_t result = pico_interface::unpack_Velocity(
        command, vel);
    if (result != pico_interface::E_MSG_SUCCESS) 
    {
        printf("$ERR: %s\n", MESSAGE_GET_ERROR(result).c_str());
        return false;
    }

    this->setVelocities(vel.motor_1_velocity, vel.motor_2_velocity);
    this->last_cmd_time = get_absolute_time();
    
    return true;
}

void
MotorControl::setVelocities(
    float motor_1_vel, 
    float motor_2_vel)
{
    mutex_enter_blocking(&this->motor_1.mtx);
    this->motor_1.desired_velocity = motor_1_vel;
    mutex_exit(&this->motor_1.mtx);

    mutex_enter_blocking(&this->motor_2.mtx);
    this->motor_2.desired_velocity = motor_2_vel;
    mutex_exit(&this->motor_2.mtx);

    printf("set velocities to: %f, %f\n", this->motor_1.desired_velocity, this->motor_2.desired_velocity);
}

std::tuple<float, float>
MotorControl::getDesiredVelocities()
{
    mutex_enter_blocking(&this->motor_1.mtx);
    float motor_1_vel = this->motor_1.desired_velocity;
    mutex_exit(&this->motor_1.mtx);
    
    mutex_enter_blocking(&this->motor_2.mtx);
    float motor_2_vel = this->motor_2.desired_velocity;
    mutex_exit(&this->motor_2.mtx);

    printf("get velocities: %f, %f\n", this->motor_1.desired_velocity, this->motor_2.desired_velocity);

    return std::tuple<float, float>(motor_1_vel, motor_2_vel);
}

std::tuple<float, float>
MotorControl::getCurrentVelocities()
{
    mutex_enter_blocking(&this->motor_1.mtx);
    float motor_1_vel = this->motor_1.current_velocity;
    mutex_exit(&this->motor_1.mtx);
    
    mutex_enter_blocking(&this->motor_2.mtx);
    float motor_2_vel = this->motor_2.current_velocity;
    mutex_exit(&this->motor_2.mtx);

    return std::tuple<float, float>(motor_1_vel, motor_2_vel);
}

void
MotorControl::report()
{
    pico_interface::Msg_Velocity vel;
    vel.motor_1_velocity = this->encoder1->getAngularVel();
    vel.motor_2_velocity = this->encoder2->getAngularVel();

    std::string out;
    pico_interface::message_error_t result = pico_interface::pack_Velocity(
        vel, pico_interface::MSG_ID_VELOCITY_STATUS, out);
    if (result != pico_interface::E_MSG_SUCCESS) {
        out = pico_interface::MESSAGE_GET_ERROR(result);
    }

    printf("FlAG == %s\n", (this->FLAG) ? "TRUE" : "FALSE");

    printf(out.c_str());
}
