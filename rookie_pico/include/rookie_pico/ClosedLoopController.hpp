#ifndef CLOSED_LOOP_CONTROLLER_HPP
#define CLOSED_LOOP_CONTROLLER_HPP

// CPP headers
#include <algorithm>
#include <memory>
#include <stdlib.h>

// Pico headers
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "pico/mutex.h"

// Project headers
#include <rookie_pico/Encoders.hpp>
#include <rookie_pico/Flag.hpp>

class ClosedLoopController 
{
public:

    ClosedLoopController(
        std::shared_ptr<MDD10A> controller,
        std::shared_ptr<Encoder> encoder1,
        std::shared_ptr<Encoder> encoder2,
        Flag* flag);

    // bool init();
    
    /**
     * \brief Set the desired velocity (in rads/sec) of our motors.
     * 
     * Values set here will be applied on the next cycle.
     * 
     * \param[in] motor_1_vel desired velocity (rads/sec) of motor 1
     * \param[in] motor_2_vel desired velocity (rads/sec) of motor 2
     */
    void setVelocities(
        float motor_1_vel, 
        float motor_2_vel);

    std::tuple<float, float> getDesiredVelocities();

    std::tuple<float, float> getCurrentVelocities();

    void report();

    bool onCycle();

    bool handleCommand(const std::string command);

    std::string getStatus() { return this->status; }

protected:
private:

    std::string status;

    absolute_time_t last_cmd_time = nil_time;

    typedef struct Motor {
        mutex_t mtx;
        float desired_velocity = 0.0;
        float current_velocity = 0.0;
    } Motor;

    Motor motor_1;
    Motor motor_2;

    std::shared_ptr<MDD10A> controller;
    std::shared_ptr<Encoder> encoder1;
    std::shared_ptr<Encoder> encoder2;

    // bool FLAG = false;
    Flag* FLAG;

}; // class ClosedLoopController

#endif // CLOSED_LOOP_CONTROLLER_HPP