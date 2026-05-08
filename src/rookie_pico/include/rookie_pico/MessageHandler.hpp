#ifndef MESSAGE_HANDLER_HPP
#define MESSAGE_HANDLER_HPP

#include <cstdint>

class IMessageHandler
{
public:

    enum class Result
    {
        // This msg_id is not handled by this IMessageHandler
        NOT_MINE,
        // Message handled successfully
        OK,
        // Error when handling message
        ERROR
    };

    virtual Result handle(uint32_t msg_id, const uint8_t* payload, std::size_t len) = 0;
}; // class IMessageHandler

#endif // MESSAGE_HANDLER_HPP