
// C headers
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

// CPP headers
#include <optional>

// ROS headers
#include <rclcpp/rclcpp.hpp>

#include <mobility_driver/SerialPort.hpp>

namespace mobility_driver
{

SerialPort::~SerialPort()
{
    close(this->serial_port);
}


bool
SerialPort::configure(int frequency)
{
    this->serial_port = ::open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_port < 0) 
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("serial_port"),
            "Error opening serial port: <" << errno << "> == " << strerror(errno));
        return false;
    }

    struct termios tty;
    if (tcgetattr(serial_port, &tty) != 0) 
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("serial_port"),
            "Error getting terminal settings: <" << errno << "> == " << strerror(errno));
        return false;
    }
    
    // Configure Control Modes.
    // PARITY
    tty.c_cflag &= ~PARENB; // no parity bit
    // STOP BIT
    tty.c_cflag &= ~CSTOPB; // use ONE stop bit
    // (DATA) BITS PER BYTE
    tty.c_cflag &= ~CSIZE;  // first clear...
    tty.c_cflag |= CS8;     // ... then set; 8 bits per byte
    // HW FLOW CONTROL
    tty.c_cflag &= ~CRTSCTS; // disabled
    // CREAD and CLOCAL
    tty.c_cflag |= CREAD | CLOCAL;   // allow reading, ignore control lines

    // Configure local modes.
    // CANONICAL-MODE: receive only on '\n' (no) or constantly (yes)
    tty.c_lflag &= ~ICANON; // disable canonical mode
    // ECHO: bits we send get sent back (no thanks)
    tty.c_lflag &= ~ECHO;   // Disable echo
    tty.c_lflag &= ~ECHOE;  // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    // SIGNAL CHARS (INTR, QUIT, SUSP)
    tty.c_lflag &= ~ISIG; // disable signal chars

    // Configure input modes.
    // SW FLOW CONTROL
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // disable sw flow control
    // Disable any special handling of bytes on receive
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

    // Configure output modes.
    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

    // VMIN/VTIME
    // We want non-blocking (polling) reads
    tty.c_cc[VTIME] = 0;    // Do not wait for any period of time.
    tty.c_cc[VMIN] = 0;     // No minimum amount of bytes before returning.

    // BAUDRATE
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    if (tcsetattr(serial_port, TCSANOW, &tty) != 0)
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("serial_port"),
            "Failed to set serial settings: <" << errno << "> == " << strerror(errno));
        return false;
    }

    this->frequency = frequency;
    this->configured = true;
    return true;
}

std::optional<std::string>
SerialPort::spinOnce()
{
    if (!this->isConfigured()) { return std::nullopt; }

    char buffer[256];
    char ch = '\n';

    memset(&buffer, '\0', sizeof(buffer));

    int total_bytes = 0;
    int num_bytes = ::read(this->serial_port, &ch, 1);
    while (ch != '\n')
    {
        if (num_bytes <= 0)
        {
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("serial_port"), 
                "Bytes read ==  " << num_bytes << ", errno == " << errno << ", " << strerror(errno)); 
            return std::nullopt;
        }

        if (ch == '\n' || total_bytes == (sizeof(buffer))-1) 
        {
            buffer[total_bytes] = '\0';
            break;
        }
        buffer[total_bytes++] = ch;
        // get next char
        num_bytes = ::read(this->serial_port, &ch, 1);
    }

    return std::optional<std::string>{std::string(buffer)};
}

void
SerialPort::spin()
{
    if (!this->isConfigured()) { return; }

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
        if (buffer_out.find(std::string("\n")) == std::string::npos) {
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
    if (!this->isConfigured()) { return false; }

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
