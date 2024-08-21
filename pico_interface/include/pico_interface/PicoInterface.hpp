#ifndef PICO_INTERFACE_HPP
#define PICO_INTERFACE_HPP

#include <algorithm>
#include <cstdint>
#include <map>
#include <string>
#include <sstream>

namespace pico_interface {

static int
countFields(const std::string in, [[maybe_unused]] const std::string delim = " ") {
    // There's one more field than there are delimiters
    int count = std::count_if(in.begin(), in.end(), [in](unsigned char c){ return std::isspace(c); });
    return count + 1;
}

// error type for PWM functions
typedef enum MESSAGE_ERR 
{
    E_MSG_SUCCESS = 0,
    E_MSG_DECODE_FAILURE = 1,
    E_MSG_ENCODE_FAILURE = 2,
    E_MSG_DECODE_WRONG_NUM_FIELDS = 3,
    E_MSG_DECODE_OUT_OF_BOUNDS = 4,
} message_error_t;

static const std::map<message_error_t, std::string> message_error_desc = {
    {E_MSG_SUCCESS, "Success."},
    {E_MSG_DECODE_FAILURE, "Failed to decode message."},
    {E_MSG_ENCODE_FAILURE, "Failed to encode message."},
    {E_MSG_DECODE_WRONG_NUM_FIELDS, "Failed to decode message: wrong number of fields."},
    {E_MSG_DECODE_OUT_OF_BOUNDS, "Failed to decode message: field(s) contained value that is out of bounds."},
};

static const std::string 
MESSAGE_GET_ERROR(message_error_t err_code) {
    auto it =  message_error_desc.find(err_code);
    if (it == message_error_desc.end()) {
        return "";
    }
    return it->second;
}


static const std::string MSG_ID_MOTORS_CMD = "$MTR.C";
typedef struct Msg_MotorsCommand {

    enum class DIRECTION : uint8_t {
        FORWARD = 0,
        REVERSE = 1,
    };

    DIRECTION motor_1_direction;
    uint8_t motor_1_pwm;

    DIRECTION motor_2_direction;
    uint8_t motor_2_pwm;

} MOTORS_CMD;

static message_error_t
pack_MotorsCommand(
    Msg_MotorsCommand& msg,
    std::string& str)
{
    std::stringstream ss;

    ss << MSG_ID_MOTORS_CMD << " ";
    ss << std::to_string(static_cast<uint8_t>(msg.motor_1_direction)) << " ";
    ss << std::to_string(msg.motor_1_pwm) << " ";
    ss << std::to_string(static_cast<uint8_t>(msg.motor_2_direction)) << " ";
    ss << std::to_string(msg.motor_2_pwm) << "\n";

    str = ss.str();

    return MESSAGE_ERR::E_MSG_SUCCESS;
};

static message_error_t
unpack_MotorsCommand(
    const std::string msg,
    Msg_MotorsCommand& motors_cmd) 
{
    std::stringstream ss(msg);
    
    if (countFields(msg) != 4) {
        return MESSAGE_ERR::E_MSG_DECODE_WRONG_NUM_FIELDS;
    }

    std::string token;
    ss >> token;
    motors_cmd.motor_1_direction = 
        static_cast<Msg_MotorsCommand::DIRECTION>(std::atoi(token.c_str()));
    
    ss >> token;
    motors_cmd.motor_1_pwm = static_cast<uint8_t>(std::atoi(token.c_str()));
    if (motors_cmd.motor_1_pwm > 100) {
        return MESSAGE_ERR::E_MSG_DECODE_OUT_OF_BOUNDS;
    }

    ss >> token;
    motors_cmd.motor_2_direction = 
        static_cast<Msg_MotorsCommand::DIRECTION>(std::atoi(token.c_str()));
    
    ss >> token;
    motors_cmd.motor_2_pwm = static_cast<uint8_t>(std::atoi(token.c_str()));
    if (motors_cmd.motor_1_pwm > 100) {
        return MESSAGE_ERR::E_MSG_DECODE_OUT_OF_BOUNDS;
    }

    // Make sure there's nothing left over...
    if (ss.rdbuf()->in_avail() != 0) {
        return MESSAGE_ERR::E_MSG_DECODE_WRONG_NUM_FIELDS;
    }

    return MESSAGE_ERR::E_MSG_SUCCESS;
}


static const std::string MSG_ID_VELOCITY_CMD = "$VEL.C";
static const std::string MSG_ID_VELOCITY_STATUS = "$VEL.S";
typedef struct Msg_Velocity {
    float motor_1_velocity = 0.0; // rads/sec
    float motor_2_velocity = 0.0; // rads/sec
} Msg_Velocity;

static message_error_t
pack_Velocity(
    Msg_Velocity& msg,
    const std::string header,
    std::string& str) 
{
    std::stringstream ss;

    if (header != MSG_ID_VELOCITY_CMD && 
        header != MSG_ID_VELOCITY_STATUS) {
        return E_MSG_ENCODE_FAILURE;
    }

    ss << header << " ";
    ss << msg.motor_1_velocity << " ";
    ss << msg.motor_2_velocity << "\n";

    str = ss.str();
    return E_MSG_SUCCESS;
}

static message_error_t
unpack_Velocity(
    const std::string msg,
    Msg_Velocity& vel) 
{
    std::stringstream ss(msg);

    if (countFields(msg) != 2) { return E_MSG_DECODE_WRONG_NUM_FIELDS; }

    std::string token;
    ss >> token;
    vel.motor_1_velocity = std::stof(token);
    ss >> token;
    vel.motor_2_velocity = std::stof(token);

    return E_MSG_SUCCESS;
}

} // namespace pico_interface

#endif // PICO_INTERFACE_HPP