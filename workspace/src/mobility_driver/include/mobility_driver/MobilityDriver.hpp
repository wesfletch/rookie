#ifndef MOBILITY_DRIVER_HPP
#define MOBILITY_DRIVER_HPP

// CPP headers
#include <optional>

// ROS
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>

// Project specific
#include <mobility_driver/SerialPort.hpp>
#include <pico_interface/PicoInterface.hpp>


namespace mobility_driver
{

static const std::string DEFAULT_NODE_NAME = "mobility_driver";
static const std::string DEFAULT_TOPIC_PICO_OUT = "pico_out";
static const std::string DEFAULT_SERIAL_DEVICE_NAME = "/dev/ttyACM0";

// TODO: Spin rate seems really high just for polling an MCU (which publishes at 50Hz max),
// but if the spin rate is too low we can hang...
static const int DEFAULT_SPIN_RATE = 200;


// TODO: This is an annoying bodge. Can I do this in a way that isn't so tightly coupled
// to the interface file? Or at least genericize the interface file?
enum class MobilityState 
{
    STANDBY = static_cast<int>(pico_interface::Msg_SystemState::STATE::STANDBY),
    ESTOP = static_cast<int>(pico_interface::Msg_SystemState::STATE::ESTOP),
    ERROR = static_cast<int>(pico_interface::Msg_SystemState::STATE::ERROR),
    READY = static_cast<int>(pico_interface::Msg_SystemState::STATE::READY), // TODO: This and standby are maybe redundant
    TEST = static_cast<int>(pico_interface::Msg_SystemState::STATE::TEST),
};

struct MessageData {
    std::string prefix;
    std::string body;
    rclcpp::Time stamp;

    std::string str()
    {
        std::stringstream ss;
        ss << "PREFIX: '" << this->prefix << "', ";
        ss << "BODY: '" << this->body << "', ";
        ss << "TIMESTAMP: '" << this->stamp.nanoseconds() << "'" << std::endl;
    
        return ss.str();
    }
};


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

    std::optional<MessageData> splitMessage(std::string_view message)
    {
        size_t posEndOfPrefix = message.find_first_of(pico_interface::DELIM);
        if (posEndOfPrefix == std::string::npos)
        {
            std::stringstream ss;
            ss << "Failed to get message prefix: `" << message << "`" << std::endl;
            RCLCPP_DEBUG_STREAM(this->_node->get_logger(), ss.str());
            return std::nullopt;
        }
    
        // split into prefix + the rest
        std::string prefix(message, 0, posEndOfPrefix);
        std::string body(message, posEndOfPrefix + 1, std::string::npos);
        rclcpp::Time stamp = this->_node->now();

        return MessageData{prefix, body, stamp};
    };


protected:
private:

    rclcpp::Node::SharedPtr _node;

    MobilityState state = MobilityState::STANDBY;

    const int spin_rate;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pico_out_pub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub;
        
    SerialPort serial_port;

    rclcpp::TimerBase::SharedPtr heartbeat_timer;
    uint32_t heartbeat_seq = 0;

    float desired_vel_left = 0.0;
    float desired_vel_right = 0.0;
};

} // namespace mobility_driver

#endif // MOBILITY_DRIVER_HPP