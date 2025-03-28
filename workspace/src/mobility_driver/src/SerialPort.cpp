
// C headers
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdexcept>
#include <termios.h>
#include <unistd.h>

// CPP headers
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
    this->serial_port = ::open(this->device_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_port < 0) 
    {
        std::stringstream err;
        err << "Error opening serial port " << this->device_name << ": <" << errno << "> == " << strerror(errno);
        throw std::runtime_error(err.str());
    }

    struct termios tty;
    if (tcgetattr(serial_port, &tty) != 0) 
    {
        std::stringstream err;
        err << "Error getting terminal settings: <" << errno << "> == " << strerror(errno);
        throw std::runtime_error(err.str());
    }
    
    this->configure_tty(&tty); 
    
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
SerialPort::configure_tty(struct termios* tty)
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

    int num_bytes = ::read(this->serial_port, &ch, 1);
    while (ch != '\n')
    {
        if (num_bytes < 0)
        {
            // We got some sort of read error.
            RCLCPP_WARN_STREAM(rclcpp::get_logger("serial_port"), 
                "Bytes read ==  " << num_bytes << ", errno == " << errno << ", " << strerror(errno)); 
            return std::nullopt;
        }
        else if (num_bytes > 0)
        {
            buffer.push_back(ch);
        }
        else 
        {
            // We got nothing; turf and come back later.
            break;
        }

        // get next char
        num_bytes = ::read(this->serial_port, &ch, 1);
    }

    return std::optional<std::string>{buffer};
}

void
SerialPort::spin()
{
    char buffer[256];
    memset(&buffer, '\0', sizeof(buffer));

    std::string buffer_out;

    rclcpp::Rate loop_rate(this->frequency);
    while (rclcpp::ok())
    {
        int num_bytes = ::read(this->serial_port, &buffer, 256);
        if (num_bytes == 0) 
        {
            continue; 
        }
        else if (num_bytes < 0) 
        { 
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("serial_port"), 
                "ERR " << errno << ", " << strerror(errno)); 
            continue; 
        }

        // TODO: trying to do string accumulation here, but it's not going very well for me.
        buffer_out += std::string(buffer);
        if (buffer_out.find(std::string("\n")) == std::string::npos) 
        {
            continue;
        }
        buffer[num_bytes] = '\0';
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("serial_port"), 
            "read: <" << num_bytes << ">, \"" << std::string(buffer) << "\"");

        loop_rate.sleep();
    }
}


bool
SerialPort::write(
    std::string out)
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
