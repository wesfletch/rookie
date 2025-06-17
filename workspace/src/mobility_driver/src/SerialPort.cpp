
// C headers
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

// CPP headers
#include <stdexcept>
#include <stop_token>
#include <optional>

// ROS headers
#include <rclcpp/rclcpp.hpp>

#include <mobility_driver/SerialPort.hpp>

namespace mobility_driver
{


SerialPort::SerialPort(
    std::string_view device_name,
    const int& frequency)
    :
    device_name(std::string(device_name)),
    frequency(frequency)
{
    // Open the serial device
    this->serial_port = ::open(this->device_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_port < 0) 
    {
        std::stringstream err;
        err << "Error opening serial port " << this->device_name << ": <" 
            << errno << "> == " << strerror(errno);
        throw std::runtime_error(err.str());
    }

    // Configure our TTY
    struct termios tty;
    if (tcgetattr(serial_port, &tty) != 0) 
    {
        std::stringstream err;
        err << "Error getting terminal settings: <" << errno << "> == " << strerror(errno);
        throw std::runtime_error(err.str());
    }

    this->_configure_tty(&tty); 
    
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0)
    {
        std::stringstream err;
        err << "Failed to configure TTY: <" << errno << "> == " << strerror(errno);
        throw std::runtime_error(err.str());
    }
}


SerialPort::~SerialPort()
{
    close(this->serial_port);
}


void
SerialPort::_configure_tty(struct termios* tty)
{
    // Configure Control Modes.
    tty->c_cflag &= ~PARENB; // No parity bit
    tty->c_cflag &= ~CSTOPB; // use ONE stop bit
    // (DATA) BITS PER BYTE
    tty->c_cflag &= ~CSIZE;  // first clear...
    tty->c_cflag |= CS8;     // ... then set; 8 bits per byte
    
    // Flow control
    tty->c_cflag &= ~CRTSCTS; // HW flow control disabled
    tty->c_iflag &= ~(IXON | IXOFF | IXANY); // SW flow control disabled

    // Configure local modes.
    tty->c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // disable canonical mode
    tty->c_oflag &= ~OPOST;

    // Configure input modes.
    tty->c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

    // Configure output modes.
    tty->c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty->c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

    // We want non-blocking (polling) reads
    tty->c_cc[VTIME] = 0;    // Do not wait for any period of time.
    tty->c_cc[VMIN] = 0;     // No minimum amount of bytes before returning.

    // BAUDRATE, IN and OUT
    cfsetspeed(tty, B115200);
}


std::optional<std::string>
SerialPort::spinOnce()
{
    std::string buffer;
    char ch = '\n';

    // First read
    int num_bytes = ::read(this->serial_port, &ch, 1);
    if (num_bytes == 0) { 
        // Nothing's wrong, but we didn't get anything.
        return std::nullopt; 
    }

    // Read the rest of the chars
    while (ch != '\n')
    {
        // We got some sort of read error.
        if (num_bytes < 0)
        {
            RCLCPP_WARN_STREAM(rclcpp::get_logger("serial_port"), 
                "Bytes read ==  " << num_bytes << ", errno == " << errno << ", " << strerror(errno)); 
            return std::nullopt;
        }
        // Happy path: we have bytes to read.
        else if (num_bytes > 0)
        {
            buffer.push_back(ch);
        }
        // We got nothing; turf and come back later.
        else 
        {
            break;
        }

        // get next char
        num_bytes = ::read(this->serial_port, &ch, 1);
    }

    // This shouldn't actually happen...
    if (buffer.size() == 0) { return std::nullopt; }

    return std::optional<std::string>{buffer};
}


void 
SerialPort::start_jthread()
{
    // TODO: idempotency, or maybe it doesn't matter.

    // Thread starts running when jthread is initialized, will be joined automatically
    // on destruction.
    this->thread = std::jthread(std::bind_front(&SerialPort::spin_jthread, this));
}
    
void 
SerialPort::stop_jthread()
{
    this->thread.request_stop();
}

void
SerialPort::spin_jthread(std::stop_token stop_token)
{
    std::optional<std::string> in_string;
    while (!stop_token.stop_requested()) 
    {
        // READ
        in_string = this->spinOnce();
        if (in_string) 
        {
            {
                std::unique_lock<std::mutex> lock_inbound(this->mtx_inbound);
                this->inbound.push(*in_string);
            }
        }
        
        // WRITE
        {
            std::unique_lock<std::mutex> lock_outbound(this->mtx_outbound);
            if (!this->outbound.empty()) 
            {
                if (this->_write(outbound.front()))
                {
                    // If we wrote the message successfully, then we can pop
                    // it off the queue. Otherwise, try again on the next spin. 
                    outbound.pop();
                }
            }
        }
    }
}

void 
SerialPort::push(const std::string_view message)
{
    std::unique_lock<std::mutex> lock_outbound(this->mtx_outbound);

    this->outbound.push(std::string(message));
}

std::optional<std::string>
SerialPort::pop()
{
    std::unique_lock<std::mutex> lock_inbound(this->mtx_inbound, std::defer_lock);
    if (!lock_inbound.try_lock()) 
    {
        return std::nullopt;
    }

    if (this->inbound.empty())
    {
        return std::nullopt;
    }
    
    std::string inbound_str = this->inbound.front();
    this->inbound.pop();

    return inbound_str;
}

bool
SerialPort::_write(std::string out)
{
    RCLCPP_INFO_STREAM(rclcpp::get_logger("serial_port"), "TX: " << out);

    uint num_bytes = ::write(this->serial_port, out.c_str(), out.size());
    if (num_bytes != out.size())
    {
        if (errno == EAGAIN || errno == EWOULDBLOCK)
        {
            // OUTPUT BUFFER IS FULL?
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("serial_port"), 
                "Output buffer full?");
        } 
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("serial_port"), 
            "FAILED TO WRITE <" << errno << "> == " << strerror(errno));
        return false;
    }

    return true;
}

} // namespace mobility_driver
