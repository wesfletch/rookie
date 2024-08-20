#ifndef COMMANDS_HPP
#define COMMANDS_HPP


#include <functional>
#include <map>
#include <string>
#include <sstream>

// #include <rookie_pico/Definitions.hpp>
#include <rookie_pico/PicoInterface.hpp>


bool
handle_Motors(
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

    // TODO: should actually do something
    return true;
}

bool
handle_Velocity(const std::string input)
{
    Msg_Velocity vel = {};
    
    message_error_t status = unpack_Velocity(input, vel);
    if (status != E_MSG_SUCCESS) 
    {
        printf("$ERR: %s\n", MESSAGE_GET_ERROR(status).c_str());
        return false;
    }

    std::string out;
    status = pack_Velocity(vel, MSG_ID_VELOCITY_CMD, out);
    if (status != E_MSG_SUCCESS) 
    { 
        printf("$ERR: %s\n", MESSAGE_GET_ERROR(status).c_str());
        return false;
    }
    printf("OUT: <%s>\n", out.c_str());

    // TODO: should actually do something
    return true;
}

std::map<std::string, std::function<bool(std::string)>> command_callbacks = {
    {MSG_ID_MOTORS_CMD, handle_Motors},
    {MSG_ID_VELOCITY_CMD, handle_Velocity}
};

using CommandCallbacks = std::map<std::string, std::function<bool(std::string)>>;


#endif // COMMANDS_HPP