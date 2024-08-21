#ifndef MOBILITY_DRIVER_HPP
#define MOBILITY_DRIVER_HPP

// ROS
#include <rclcpp/rclcpp.hpp>

// ROS messages
#include <std_msgs/msg/string.hpp>

namespace mobility_driver
{

class SerialPort
{
public:

    bool configure();
    
    bool isConfigured() { return configured; }

protected:
private:
    int serial_port;

    bool configured = false;
}; // class SerialPort


class MobilityDriver : public rclcpp::Node
{
public:
    MobilityDriver();
    
    void run();

protected:
private:


    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;

};

} // namespace mobility_driver

#endif // MOBILITY_DRIVER_HPP