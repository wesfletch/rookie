#ifndef MOBILITY_DRIVER_HPP
#define MOBILITY_DRIVER_HPP

// ROS
#include <rclcpp/rclcpp.hpp>

// ROS messages
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>


#include <mobility_driver/SerialPort.hpp>

namespace mobility_driver
{

class MobilityDriver
{
public:
    MobilityDriver();
    
    void run();

    void pushHeartbeat();

    void pushVelocity();

    void cmdVelCallback(
        geometry_msgs::msg::Twist::UniquePtr msg);

protected:
private:

    rclcpp::Node::SharedPtr _node;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pico_out_pub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub;


    SerialPort serial_port;
    std::mutex serial_port_mtx;

    rclcpp::TimerBase::SharedPtr heartbeat_timer;
    uint32_t heartbeat_seq = 0;

    float desired_vel_left = 0.0;
    float desired_vel_right = 0.0;

};

} // namespace mobility_driver

#endif // MOBILITY_DRIVER_HPP