/**
 * TODO: If I'm willing to give up human-readable/-writable messages on the wire,
 * I could just use an off-the-shelf solution (like flatbuffers) instead of having
 * to roll my own brittle quasi-serialization solution here. Might be worth it...
 */
#ifndef PICO_INTERFACE_HPP
#define PICO_INTERFACE_HPP

#include <algorithm>
#include <cstdint>
#include <map>
#include <string>
#include <sstream>

namespace pico_interface {

static const std::string DELIM = " ";
static const std::string END = "\n";


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


static const std::string MSG_ID_HEARTBEAT = "$HBT";
typedef struct Msg_Heartbeat {
    uint32_t seq;
} HEARTBEAT;

static message_error_t
pack_HeartbeatMessage(
    Msg_Heartbeat& heartbeat,
    std::string& str)
{
    std::stringstream ss;

    ss << MSG_ID_HEARTBEAT << DELIM;
    ss << heartbeat.seq << END;
    str = ss.str();

    return MESSAGE_ERR::E_MSG_SUCCESS;
}

static message_error_t
unpack_Heartbeat(
    const std::string msg,
    Msg_Heartbeat& heartbeat)
{
    std::stringstream ss(msg);

    if (pico_interface::countFields(msg) != 1) {
        return MESSAGE_ERR::E_MSG_DECODE_WRONG_NUM_FIELDS;
    }

    std::string token;
    ss >> token;
    heartbeat.seq = 
        static_cast<uint32_t>(std::atoi(token.c_str()));

    // Make sure there's nothing left over...
    if (ss.rdbuf()->in_avail() != 0) {
        return MESSAGE_ERR::E_MSG_DECODE_WRONG_NUM_FIELDS;
    }

    return MESSAGE_ERR::E_MSG_SUCCESS;
}

static const std::string MSG_ID_ACK = "$ACK";
typedef struct Msg_Ack {

    std::string header;
    std::string fields;

    enum class STATUS : uint8_t {
        SUCCESS = 0,
        FAILURE = 1
    };

    Msg_Ack::STATUS status;

} Msg_Ack;

static message_error_t
pack_Ack(
    Msg_Ack& ack,
    std::string& str)
{
    std::stringstream ss;

    ss << MSG_ID_ACK << DELIM;
    ss << ack.header << DELIM;
    ss << ack.fields << DELIM;
    ss << std::to_string(static_cast<uint8_t>(ack.status)) << END;
    str = ss.str();

    return MESSAGE_ERR::E_MSG_SUCCESS;
}

static message_error_t
unpack_Ack(
    const std::string msg,
    Msg_Ack& ack)
{
    std::stringstream ss(msg);

    if (pico_interface::countFields(msg) != 3) {
        return MESSAGE_ERR::E_MSG_DECODE_WRONG_NUM_FIELDS;
    }

    std::string token;
    ss >> token;
    ack.header = token.c_str();

    ss >> token;
    ack.fields = token.c_str();

    ss >> token;
    ack.status = static_cast<Msg_Ack::STATUS>(atoi(token.c_str()));

    // Make sure there's nothing left over...
    if (ss.rdbuf()->in_avail() != 0) {
        return MESSAGE_ERR::E_MSG_DECODE_WRONG_NUM_FIELDS;
    }

    return MESSAGE_ERR::E_MSG_SUCCESS;
}


static const std::string MSG_ID_SYSTEM_STATE_CMD = "$SYS.C";
static const std::string MSG_ID_SYSTEM_STATE_STATUS = "$SYS.S";
typedef struct Msg_SystemState {

    enum class STATE : uint8_t {
        STANDBY = 0,
        ESTOP = 1,
        ERROR = 2,
        READY = 3,
        TEST = 4,
    };

    STATE state = STATE::STANDBY;

    std::string status = "";
} Msg_SystemState;

static message_error_t
pack_SystemState(Msg_SystemState& msg, const std::string header, std::string& str)
{
    std::stringstream ss;

    if ((header != MSG_ID_SYSTEM_STATE_CMD) && (header != MSG_ID_SYSTEM_STATE_STATUS))
    {
        return E_MSG_ENCODE_FAILURE;
    }

    ss << header << DELIM;
    ss << std::to_string(static_cast<uint8_t>(msg.state)) << DELIM;
    ss << msg.status << END;

    str = ss.str();
    return E_MSG_SUCCESS;
}

static message_error_t
unpack_SystemState(const std::string msg, Msg_SystemState& state)
{
    std::stringstream ss(msg);

    if (countFields(msg) != 2) { return E_MSG_DECODE_WRONG_NUM_FIELDS; }

    std::string token;
    ss >> token;
    state.state = static_cast<Msg_SystemState::STATE>(std::atoi(token.c_str()));
    ss >> token;
    state.status = token;
    
    return E_MSG_SUCCESS;
};


static const std::string MSG_ID_MOTORS_CMD = "$MTR.C";
static const std::string MSG_ID_MOTORS_STATUS = "$MTR.S";
typedef struct Msg_Motors {

    enum class DIRECTION : uint8_t {
        FORWARD = 0,
        REVERSE = 1,
    };

    DIRECTION motor_1_direction;
    uint8_t motor_1_pwm;

    DIRECTION motor_2_direction;
    uint8_t motor_2_pwm;

} Msg_Motors;

static message_error_t
pack_Motors(
    Msg_Motors& msg,
    const std::string header,
    std::string& str)
{
    std::stringstream ss;

    if (header != MSG_ID_MOTORS_CMD && 
        header != MSG_ID_MOTORS_STATUS) {
        return MESSAGE_ERR::E_MSG_ENCODE_FAILURE;
    }

    ss << header << DELIM;
    ss << std::to_string(static_cast<uint8_t>(msg.motor_1_direction)) << DELIM;
    ss << std::to_string(msg.motor_1_pwm) << DELIM;
    ss << std::to_string(static_cast<uint8_t>(msg.motor_2_direction)) << DELIM;
    ss << std::to_string(msg.motor_2_pwm) << END;

    str = ss.str();

    return MESSAGE_ERR::E_MSG_SUCCESS;
};

static message_error_t
unpack_Motors(
    const std::string msg,
    Msg_Motors& motors) 
{
    std::stringstream ss(msg);
    
    if (countFields(msg) != 4) {
        return MESSAGE_ERR::E_MSG_DECODE_WRONG_NUM_FIELDS;
    }

    std::string token;
    ss >> token;
    motors.motor_1_direction = 
        static_cast<Msg_Motors::DIRECTION>(std::atoi(token.c_str()));
    
    ss >> token;
    motors.motor_1_pwm = static_cast<uint8_t>(std::atoi(token.c_str()));
    if (motors.motor_1_pwm > 100) {
        return MESSAGE_ERR::E_MSG_DECODE_OUT_OF_BOUNDS;
    }

    ss >> token;
    motors.motor_2_direction = 
        static_cast<Msg_Motors::DIRECTION>(std::atoi(token.c_str()));
    
    ss >> token;
    motors.motor_2_pwm = static_cast<uint8_t>(std::atoi(token.c_str()));
    if (motors.motor_1_pwm > 100) {
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

    ss << header << DELIM;
    ss << msg.motor_1_velocity << DELIM;
    ss << msg.motor_2_velocity << END;

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