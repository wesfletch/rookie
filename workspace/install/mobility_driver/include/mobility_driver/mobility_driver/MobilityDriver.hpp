#ifndef MOBILITY_DRIVER_HPP
#define MOBILITY_DRIVER_HPP

// ROS
#include <rclcpp/rclcpp.hpp>

// ROS messages
#include <std_msgs/msg/string.hpp>

namespace mobility_driver
{

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