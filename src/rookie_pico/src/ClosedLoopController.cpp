#include <string>
#include <utility>

#include <rookie_pico/ClosedLoopController.hpp>
#include <rookie_pico/Motors.hpp>
#include <rookie_pico/RobotConfig.hpp>

#include <pico_interface/PicoInterface.hpp>
#include <pico_interface/protos/Velocity.pb.hpp>

void
twist_to_wheel_velocities(const Velocity& vel, float& left_vel, float& right_vel)
{
    left_vel = (vel.linear_vel - vel.angular_vel * (robot::wheels::WHEELBASE_M / 2)) /
               robot::wheels::WHEEL_RADIUS_M;
    right_vel = (vel.linear_vel + vel.angular_vel * (robot::wheels::WHEELBASE_M / 2)) /
                robot::wheels::WHEEL_RADIUS_M;
}

void
wheel_velocities_to_twist(Velocity& vel, const float& left_vel, const float& right_vel)
{
    vel.linear_vel = (right_vel + left_vel) / 2;
    vel.has_linear_vel = true;
    vel.angular_vel = (right_vel - left_vel) / robot::wheels::WHEELBASE_M;
    vel.has_angular_vel = true;
}

ClosedLoopController::ClosedLoopController(
    std::shared_ptr<MDD10A> controller,
    std::shared_ptr<Encoder> encoder1,
    std::shared_ptr<Encoder> encoder2,
    Flag* flag,
    OutboundQueue* outbound)
{
    this->controller = std::move(controller);
    this->encoder1 = std::move(encoder1);
    this->encoder2 = std::move(encoder2);
    this->_outbound = outbound;

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
    else if (
        absolute_time_diff_us(delayed_by_ms(this->last_cmd_time, 1000), get_absolute_time()) < 0)
    {
        // Timeout condition
        this->setVelocities(0.0, 0.0);
        this->status = "Velocity command time-out. Velocities zeroed.";
    }

    this->status = "OK";

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
    double motor_1_rpm = std::abs((motor_1_speed_rads * 60.0f) / (2 * M_PI));
    double motor_2_rpm = std::abs((motor_2_speed_rads * 60.0f) / (2 * M_PI));

    // get the PWM value that would be necessary to reach this speed
    int motor_1_pwm = static_cast<int>((motor_1_rpm / robot::motor::MOTOR_MAX_RPM) * 100);
    int motor_2_pwm = static_cast<int>((motor_2_rpm / robot::motor::MOTOR_MAX_RPM) * 100);

    // clamp to our valid pwm range
    motor_1_pwm = std::clamp(motor_1_pwm, 0, 100);
    motor_2_pwm = std::clamp(motor_2_pwm, 0, 100);

    this->controller->setMotors(motor_1_dir, motor_1_pwm, motor_2_dir, motor_2_pwm);

    return true;
}

IMessageHandler::Result
ClosedLoopController::handle(uint32_t msg_id, const uint8_t* payload, std::size_t len)
{
    // If our flag is not OK, don't do anything with this.
    if (!(*this->FLAG))
    {
        this->status = "Ignoring command because: Flag disabled.\n";
        return IMessageHandler::Result::ERROR;
    }

    pico_interface::message_error_t err;
    switch (msg_id)
    {
        case nanopb::MessageDescriptor<VelocityCommand>::msgid():
            VelocityCommand cmd;
            err = pico_interface::decode_payload(cmd, payload, len);
            if (err != pico_interface::E_MSG_SUCCESS)
            {
                return IMessageHandler::Result::ERROR;
            }

            if (!this->handleVelocityCommand(cmd))
            {
                return IMessageHandler::Result::ERROR;
            };
            break;
        default:
            return IMessageHandler::Result::NOT_MINE;
    }

    return IMessageHandler::Result::OK;
}

bool
ClosedLoopController::handleVelocityCommand(const VelocityCommand& cmd)
{
    // Decompose our twist message into left/right wheel velocities
    float left_vel = 0.0f;
    float right_vel = 0.0f;
    twist_to_wheel_velocities(cmd.commanded_velocity, left_vel, right_vel);

    this->setVelocities(left_vel, right_vel);
    this->last_cmd_time = get_absolute_time();

    // Enqueue a response
    VelocityResponse resp = VelocityResponse_init_zero;

    Velocity vel = Velocity_init_zero;
    const auto [vel_left, vel_right] = this->getCurrentVelocities();
    wheel_velocities_to_twist(vel, vel_left, vel_right);
    resp.has_current_velocity = true;
    resp.current_velocity = vel;

    resp.seq = cmd.seq;
    resp.success = true;
    resp.has_status = true;
    strncpy(resp.status, this->status.c_str(), sizeof(resp.status) - 1);

    this->_outbound->try_enqueue(resp);
    return true;
}

void
ClosedLoopController::setVelocities(float motor_1_vel, float motor_2_vel)
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
    VelocityReport report = VelocityReport_init_zero;
    Velocity vel = Velocity_init_zero;

    wheel_velocities_to_twist(
        vel, this->encoder1->getAngularVel(), this->encoder2->getAngularVel());

    report.velocity = vel;
    report.has_velocity = true;

    this->_outbound->try_enqueue(report);

    // Trigger our controller to report as well.
    this->controller->report();
}