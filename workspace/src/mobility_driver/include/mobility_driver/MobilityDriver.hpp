#ifndef MOBILITY_DRIVER_HPP
#define MOBILITY_DRIVER_HPP

// ROS
#include <rclcpp/rclcpp.hpp>

// ROS messages
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace mobility_driver
{

class SerialPort
{
public:
    ~SerialPort();

    bool configure();
    
    bool isConfigured() { return configured; }

    std::optional<std::string> spinOnce();
    void spin();

    bool write(std::string out);

protected:
private:

    /**
     * NOTE: `serial_port` not usable unless isConfigured();
     */
    int serial_port;

    bool configured = false;
    
}; // class SerialPort


// class MobilityDriver : public rclcpp::Node
class MobilityDriver
{
public:
    MobilityDriver();
    
    void run();

    void pushHeartbeat();

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

};

} // namespace mobility_driver

#endif // MOBILITY_DRIVER_HPP