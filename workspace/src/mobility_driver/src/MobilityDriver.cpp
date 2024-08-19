
// CPP headers
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>

// ROS headers
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

// Rookie-specific
#include <mobility_driver/MobilityDriver.hpp>




namespace mobility_driver
{


MobilityDriver::MobilityDriver()
    : Node("mobility_driver")
{
    this->publisher = this->create_publisher<std_msgs::msg::String>(
        "test_topic", 10
    );
}



bool
SerialPort::configure()
{
    int serial_port = open("/dev/ttyACM0", O_RDWR);
    if (serial_port < 0) 
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("serial_port"),
            "Error opening serial port: <" << errno << "> == " << strerror(errno));
        return false;
    }

    struct termios tty;
    if (!tcgetattr(serial_port, &tty)) 
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
    // BITS PER BYTE
    tty.c_cflag &= ~CSIZE;  // first clear...
    tty.c_cflag |= CS8;     // ... then set; 8 bits per byte
    // HW FLOW CONTROL
    tty.c_cflag &= ~CRTSCTS; // disabled
    // CREAD and CLOCAL
    tty.c_cflag |= CREAD;   // allow reading
    tty.c_cflag |= CLOCAL;  // ignore control lines

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
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feedtty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)

    // VMIN/VTIME
    // We want non-blocking (polling) reads
    tty.c_cc[VTIME] = 0;    // Do not wait for any period of time.
    tty.c_cc[VMIN] = 0;     // No minimum amount of bytes before returning.

    // BAUDRATE
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    if (!tcsetattr(serial_port, TCSANOW, &tty))
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("serial_port"),
            "Failed to set serial settings: <" << errno << "> == " << strerror(errno));
        return false;
    }

    this->configured = true;
    return true;
}



void
MobilityDriver::run()
{
    SerialPort ser;
    if (!ser.configure()) { return; }

    rclcpp::Rate loop_rate(50);
    while (rclcpp::ok()) 
    {   
        std_msgs::msg::String msg;
        msg.data = "LOOP";
        this->publisher->publish(msg);

        std::cout << "LOOP" << std::endl;
        loop_rate.sleep();
    }
}





} // namespace mobility_driver