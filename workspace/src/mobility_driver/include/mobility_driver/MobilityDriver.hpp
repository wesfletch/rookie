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

static const std::string DEFAULT_NODE_NAME = "mobility_driver";
static const std::string DEFAULT_TOPIC_PICO_OUT = "pico_out";
static const std::string DEFAULT_SERIAL_DEVICE_NAME = "/dev/ttyACM0";

// TODO: Spin rate seems really high just for polling an MCU (which publishes at 50Hz max),
// but if the spin rate is too low we can hang...
static const int DEFAULT_SPIN_RATE = 200;


class MobilityDriver
{
public:
    MobilityDriver(
        const std::string& node_name = DEFAULT_NODE_NAME,
        const std::string& output_topic = DEFAULT_TOPIC_PICO_OUT,
        const int& spin_rate = DEFAULT_SPIN_RATE,
        const std::string& serial_device_name = DEFAULT_SERIAL_DEVICE_NAME);

    void run();

    void pushHeartbeat();

    void pushVelocity();

    void cmdVelCallback(
        geometry_msgs::msg::Twist::UniquePtr msg);

protected:
private:

    rclcpp::Node::SharedPtr _node;

    const int spin_rate;

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