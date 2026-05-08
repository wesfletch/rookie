#ifndef PICO_INTERFACE_HPP
#define PICO_INTERFACE_HPP

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>

#include <pico_interface/pb.h>
#include <pico_interface/pb_decode.h>
#include <pico_interface/pb_encode.h>

namespace pico_interface
{

// Frame format: [SYNC_0: 0xAA] [SYNC_1: 0x55] [LENGTH: uint16_t LE] [TYPE: uint8_t] [PAYLOAD: LENGTH bytes]
// LENGTH is the number of payload bytes, not including the TYPE byte.
// Total frame size = FRAME_HEADER_SIZE + LENGTH.

static constexpr uint8_t SYNC_0 = 0xAA;
static constexpr uint8_t SYNC_1 = 0x55;
static constexpr size_t FRAME_HEADER_SIZE = 5;
static constexpr size_t MAX_PAYLOAD_SIZE = 128;
static constexpr size_t MAX_FRAME_SIZE = FRAME_HEADER_SIZE + MAX_PAYLOAD_SIZE;

struct Frame
{
    uint8_t buf[FRAME_HEADER_SIZE + MAX_PAYLOAD_SIZE];
    size_t len;
};

typedef enum MESSAGE_ERR
{
    E_MSG_SUCCESS = 0,
    E_MSG_ENCODE_FAILURE = 1,
    E_MSG_DECODE_FAILURE = 2,
    E_MSG_INVALID_SYNC = 3,
    E_MSG_BUFFER_TOO_SMALL = 4,
    E_MSG_INCOMPLETE_FRAME = 5,
} message_error_t;

static const std::map<message_error_t, std::string> message_error_desc = {
    { E_MSG_SUCCESS, "Success." },
    { E_MSG_ENCODE_FAILURE, "Failed to encode message." },
    { E_MSG_DECODE_FAILURE, "Failed to decode message." },
    { E_MSG_INVALID_SYNC, "Invalid sync bytes." },
    { E_MSG_BUFFER_TOO_SMALL, "Buffer too small for frame." },
    { E_MSG_INCOMPLETE_FRAME, "Buffer does not contain a complete frame." },
};

static const std::string
MESSAGE_GET_ERROR(message_error_t err_code)
{
    auto it = message_error_desc.find(err_code);
    if (it == message_error_desc.end())
    {
        return std::string();
    }
    return it->second;
}

// Encode a nanopb message into a complete LTV frame.
// The message type is read automatically from nanopb::MessageDescriptor<T>::msgid().
// On success, bytes_written is set to the total number of bytes written (header + payload).
template <typename T>
message_error_t
encode_frame(const T& msg, uint8_t* buf, size_t buf_size, size_t& bytes_written)
{
    static_assert(
        nanopb::MessageDescriptor<T>::has_msgid(),
        "Message type has no msgid — add option (nanopb_msgopt).msgid to its .proto definition");

    if (buf_size < FRAME_HEADER_SIZE)
    {
        return E_MSG_BUFFER_TOO_SMALL;
    }

    pb_ostream_t stream =
        pb_ostream_from_buffer(buf + FRAME_HEADER_SIZE, buf_size - FRAME_HEADER_SIZE);
    if (!pb_encode(&stream, nanopb::MessageDescriptor<T>::fields(), &msg))
    {
        return E_MSG_ENCODE_FAILURE;
    }

    uint16_t payload_len = static_cast<uint16_t>(stream.bytes_written);
    buf[0] = SYNC_0;
    buf[1] = SYNC_1;
    buf[2] = static_cast<uint8_t>(payload_len & 0x00FF);
    buf[3] = static_cast<uint8_t>((payload_len >> 8) & 0x00FF);
    buf[4] = static_cast<uint8_t>(nanopb::MessageDescriptor<T>::msgid());

    bytes_written = FRAME_HEADER_SIZE + payload_len;
    return E_MSG_SUCCESS;
}

// Decode the header of a complete LTV frame.
// On success, msg_id, payload, and payload_len are set to point into buf.
inline message_error_t
decode_frame(
    const uint8_t* buf,
    size_t buf_size,
    uint32_t& msg_id,
    const uint8_t*& payload,
    size_t& payload_len)
{
    if (buf_size < FRAME_HEADER_SIZE)
    {
        return E_MSG_BUFFER_TOO_SMALL;
    }

    if (buf[0] != SYNC_0 || buf[1] != SYNC_1)
    {
        return E_MSG_INVALID_SYNC;
    }

    payload_len = static_cast<size_t>(buf[2]) | (static_cast<size_t>(buf[3]) << 8);

    if (buf_size < FRAME_HEADER_SIZE + payload_len)
    {
        return E_MSG_INCOMPLETE_FRAME;
    }

    msg_id = static_cast<uint32_t>(buf[4]);
    payload = buf + FRAME_HEADER_SIZE;
    return E_MSG_SUCCESS;
}

// Decode a nanopb message from a payload buffer (as returned by decode_frame).
template <typename T>
message_error_t
decode_payload(T& msg, const uint8_t* payload, size_t payload_len)
{
    pb_istream_t stream = pb_istream_from_buffer(payload, payload_len);
    if (!pb_decode(&stream, nanopb::MessageDescriptor<T>::fields(), &msg))
    {
        return E_MSG_DECODE_FAILURE;
    }
    return E_MSG_SUCCESS;
}

} // namespace pico_interface

#endif // PICO_INTERFACE_HPP
