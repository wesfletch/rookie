
#include <rookie_pico/Motors.hpp>
#include <rookie_pico/ClosedLoopController.hpp>

#include <pico_interface/PicoInterface.hpp>

ClosedLoopController::ClosedLoopController(
    std::shared_ptr<MDD10A> controller,
    std::shared_ptr<Encoder> encoder1,
    std::shared_ptr<Encoder> encoder2,
    Flag* flag)
{
    this->controller = controller;
    this->encoder1 = encoder1;
    this->encoder2 = encoder2;

    // initialize our mutexes
    mutex_init(&this->motor_1.mtx);
    mutex_init(&this->motor_2.mtx);

    this->FLAG = flag;
}

bool
ClosedLoopController::onCycle()
{
    if (!(*this->FLAG)) 
    {
        this->setVelocities(0.0, 0.0);
        this->status = "Flag disabled. Velocities zeroed.\n";
    } 
    else if (absolute_time_diff_us(delayed_by_ms(this->last_cmd_time, 1000), get_absolute_time()) < 0) 
    {
        // Timeout condition
        this->setVelocities(0.0, 0.0);
        this->status = "Velocity command time-out. Velocities zeroed.";
    }
    else 
    {
        this->status = "OK";
    }

    // Update and read encoder speeds
    this->encoder1->update();
    this->encoder2->update();

    // Actually assert our motor velocities here
    std::tuple<float, float> desired_vels = this->getDesiredVelocities();

    // velocities that are handed to us are in rads/s
    float motor_1_speed_rads = std::get<0>(desired_vels);
    float motor_2_speed_rads = std::get<1>(desired_vels);

    // get the signs
    bool motor_1_dir = (motor_1_speed_rads >= 0);
    bool motor_2_dir = (motor_2_speed_rads >= 0);

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
ClosedLoopController::handleCommand(
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

    if (!(*this->FLAG)) { return false; }

    this->setVelocities(vel.motor_1_velocity, vel.motor_2_velocity);
    this->last_cmd_time = get_absolute_time();
    
    return true;
}

void
ClosedLoopController::setVelocities(
    float motor_1_vel, 
    float motor_2_vel)
{
    mutex_enter_blocking(&this->motor_1.mtx);
    this->motor_1.desired_velocity = motor_1_vel;
    mutex_exit(&this->motor_1.mtx);

    mutex_enter_blocking(&this->motor_2.mtx);
    this->motor_2.desired_velocity = motor_2_vel;
    mutex_exit(&this->motor_2.mtx);
}

std::tuple<float, float>
ClosedLoopController::getDesiredVelocities()
{
    mutex_enter_blocking(&this->motor_1.mtx);
    float motor_1_vel = this->motor_1.desired_velocity;
    mutex_exit(&this->motor_1.mtx);
    
    mutex_enter_blocking(&this->motor_2.mtx);
    float motor_2_vel = this->motor_2.desired_velocity;
    mutex_exit(&this->motor_2.mtx);

    return std::tuple<float, float>(motor_1_vel, motor_2_vel);
}

std::tuple<float, float>
ClosedLoopController::getCurrentVelocities()
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
ClosedLoopController::report()
{
    // Report the current velocity of the wheels as reported by our encoders.
    pico_interface::Msg_Velocity vel;
    vel.motor_1_velocity = this->encoder1->getAngularVel();
    vel.motor_2_velocity = this->encoder2->getAngularVel();

    std::string out;
    pico_interface::message_error_t result = pico_interface::pack_Velocity(
        vel, pico_interface::MSG_ID_VELOCITY_STATUS, out);
    if (result != pico_interface::E_MSG_SUCCESS) {
        out = pico_interface::MESSAGE_GET_ERROR(result);
    }
    printf(out.c_str());

    this->controller->report();
}