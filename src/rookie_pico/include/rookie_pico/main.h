#ifndef MAIN_HPP
#define MAIN_HPP

// general includes
#include <stdlib.h>
#include <string>

#include <pico_interface/PicoInterface.hpp>
#include <rookie_pico/OutboundQueue.hpp>

// TODO: where do I want these to go????
typedef enum CMD_ERROR
{
    E_CMD_SUCCESS = 0,
    E_CMD_PARSE_FAILED = 1,
    E_CMD_FAIL = 2,
} cmd_error_t;

typedef struct CMD_ERROR_DESC
{
    int16_t return_code;
    std::string message;
} cmd_error_desc;

static const cmd_error_desc cmd_error_descriptions[] = {
    { E_CMD_SUCCESS, "Success." },
    { E_CMD_PARSE_FAILED, "Failed to parse command." },
    { E_CMD_FAIL, "Command failed." },
};

volatile uint32_t heartbeat_seq = 0;
struct repeating_timer heartbeat_timer;

extern OutboundQueue outbound;

#endif // MAIN_HPP
