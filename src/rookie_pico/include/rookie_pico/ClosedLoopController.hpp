#ifndef CLOSED_LOOP_CONTROLLER_HPP
#define CLOSED_LOOP_CONTROLLER_HPP

// CPP headers
#include <memory>
#include <stdlib.h>

// Pico headers
#include <pico/mutex.h>

#include <pico_interface/protos/Velocity.pb.hpp>

// Project headers
#include <rookie_pico/Encoders.hpp>
#include <rookie_pico/Flag.hpp>
#include <rookie_pico/MessageHandler.hpp>
#include <rookie_pico/Motors.hpp>
#include <rookie_pico/OutboundQueue.hpp>

void twist_to_wheel_velocities(const Velocity& vel, float& left_vel, float& right_vel);
void wheel_velocities_to_twist(Velocity& vel, const float& left_vel, const float& right_vel);

class ClosedLoopController : public IMessageHandler
{
public:

    ClosedLoopController(
        std::shared_ptr<MDD10A> controller,
        std::shared_ptr<Encoder> encoder1,
        std::shared_ptr<Encoder> encoder2,
        Flag* flag,
        OutboundQueue* outbound);

    // bool init();

    /**
     * \brief Set the desired velocity (in rads/sec) of our motors.
     * 
     * Values set here will be applied on the next cycle.
     * 
     * \param[in] motor_1_vel desired velocity (rads/sec) of motor 1
     * \param[in] motor_2_vel desired velocity (rads/sec) of motor 2
     */
    void setVelocities(float motor_1_vel, float motor_2_vel);

    std::tuple<float, float> getDesiredVelocities();

    std::tuple<float, float> getCurrentVelocities();

    void report();

    bool onCycle();

    IMessageHandler::Result
    handle(uint32_t msg_id, const uint8_t* payload, std::size_t len) override;

    bool handleVelocityCommand(const VelocityCommand& cmd);

    std::string
    getStatus()
    {
        return this->status;
    }

protected:
private:

    std::string status;

    absolute_time_t last_cmd_time = nil_time;

    typedef struct Motor
    {
        mutex_t mtx;
        float desired_velocity = 0.0;
        float current_velocity = 0.0;
    } Motor;

    Motor motor_1;
    Motor motor_2;

    std::shared_ptr<MDD10A> controller;
    std::shared_ptr<Encoder> encoder1;
    std::shared_ptr<Encoder> encoder2;

    OutboundQueue* _outbound;

    // bool FLAG = false;
    Flag* FLAG;

}; // class ClosedLoopController

#endif // CLOSED_LOOP_CONTROLLER_HPP