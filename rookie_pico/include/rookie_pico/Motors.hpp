#ifndef MOTORS_H
#define MOTORS_H

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

// Motor Controller-specific definitions
static const std::string MDD10A_NAME = "MDD10A";
static const uint MDD10A_PWM_1_PIN = 16;
static const uint MDD10A_PWM_2_PIN = 17;
static const uint MDD10A_DIR_1_PIN = 18;
static const uint MDD10A_DIR_2_PIN = 19;


// PWM definitions
// the "wrap" is the length in cycles of a single PWM pulse
static constexpr int PWM_WRAP = 12500;
// scaling factor maps 0-100 to our pwm wrap range 0-`PWM_WRAP`
static constexpr int PWM_SCALING_FACTOR = PWM_WRAP / 100;

// Max RPM of the specific worm-gear motors I'm using
static constexpr int MOTOR_MAX_RPM = 250;
static constexpr float MOTOR_MAX_VEL_RADS = (MOTOR_MAX_RPM / 60) * (2 * M_PI); 

// Wheel parameters
static constexpr float WHEEL_DIAMETER_MM = 73.025; // 2"7/8 inches
static constexpr float WHEEL_DIAMETER_M = WHEEL_DIAMETER_MM / 1000;
static constexpr float WHEEL_RADIUS_MM = WHEEL_DIAMETER_MM / 2;
static constexpr float WHEEL_RADIUS_M = WHEEL_RADIUS_MM / 1000;

// error type for PWM functions
typedef enum PWM_ERROR 
{
    E_PWM_SUCCESS = 0,
    E_PWM_OUT_OF_RANGE = 1,
    E_PWM_SLICE_MISMATCH = 2,
    E_PWM_FAILED_SET_FUNCTION = 3,
} pwm_error_t;

static const std::map<pwm_error_t, std::string> pwm_error_descriptions = {
    {E_PWM_SUCCESS, "Success."},
    {E_PWM_OUT_OF_RANGE, "Provided PWM value outside range."},
    {E_PWM_SLICE_MISMATCH, "PWM slice mismatch"},
    {E_PWM_FAILED_SET_FUNCTION, "Failed to set PWM pin function."}
};

static std::string
get_pwm_error_desc(pwm_error_t error)
{
    auto it = pwm_error_descriptions.find(error);
    if (it == pwm_error_descriptions.end()) 
    {
        return "";
    }
    return it->second;
}

static inline uint16_t
getPwmValue(int duty_cycle)
{
    return static_cast<uint16_t>(duty_cycle * PWM_SCALING_FACTOR);
}

static inline uint8_t
getPwmDutyCycle(float velocity_rads_sec)
{
    float velocity = std::min(
        std::abs(velocity_rads_sec), 
        std::abs(MOTOR_MAX_VEL_RADS));

    // probably loss of precision here, do I care?
    uint8_t duty_cycle = (MOTOR_MAX_VEL_RADS / velocity) * 100;    
    duty_cycle = std::clamp(duty_cycle, (uint8_t)0, (uint8_t)100);

    return duty_cycle;
}

/**
 * @brief 
 * 
 * @param linear_vel 
 * @return float angular velocity in rads/sec 
 */
static inline float
linearToAngularVel(float linear_vel)
{
    return linear_vel / WHEEL_RADIUS_M;
}


class MDD10A 
{
    enum MOTOR : uint8_t {
        MOTOR_1 = 0,
        MOTOR_2 = 1
    };

public:

    /**
     * @brief Construct a new MDD10A object
     * 
     * @param dir_1_pin 
     * @param pwm_1_pin 
     * @param dir_2_pin 
     * @param pwm_2_pin 
     */
    MDD10A(
        uint dir_1_pin, uint pwm_1_pin,
        uint dir_2_pin, uint pwm_2_pin)
        :
        dir_1_pin(dir_1_pin), pwm_1_pin(pwm_1_pin),
        dir_2_pin(dir_2_pin), pwm_2_pin(pwm_2_pin){};

    /**
     * @brief Configure the MDD10A motor controllers hardware 
     * e.g., GPIO pins, PWM slices...
     * 
     * @return pwm_error_t 
     */
    pwm_error_t configure();
    
    /**
     * @brief Set the Motors object
     * 
     * @param left_dir 
     * @param left_speed 
     * @param right_dir 
     * @param right_speed 
     * @return pwm_error_t 
     */
    pwm_error_t setMotors(
        bool left_dir, int left_speed, 
        bool right_dir, int right_speed);
    
    pwm_error_t setMotor(
        MDD10A::MOTOR channel, bool dir, int speed);

    bool handleMsg_Motors(const std::string input);

protected:
private:
    const uint dir_1_pin;
    const uint pwm_1_pin;
    const uint dir_2_pin;
    const uint pwm_2_pin;

}; // class MDD10A


class MotorControl 
{
public:

    MotorControl(
        std::shared_ptr<MDD10A> controller,
        std::shared_ptr<Encoder> encoder1,
        std::shared_ptr<Encoder> encoder2);

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

    // bool* getFlag() 
    // {
    //     return &(this->FLAG);
    // }

    std::shared_ptr<Flag> getFlag()
    {}

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
    std::shared_ptr<Flag> FLAG;

}; // class MotorControl


#endif // MOTORS_H