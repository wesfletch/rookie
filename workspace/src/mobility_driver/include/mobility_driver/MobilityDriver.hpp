#ifndef MOBILITY_DRIVER_HPP
#define MOBILITY_DRIVER_HPP

// CPP headers
#include <optional>

// ROS
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

// Project specific
#include <mobility_driver/SerialPort.hpp>
#include <pico_interface/PicoInterface.hpp>


namespace mobility_driver
{

static const std::string DEFAULT_NODE_NAME = "mobility_driver";
static const std::string DEFAULT_TOPIC_PICO_OUT = "pico_out";
static const std::string DEFAULT_SERIAL_DEVICE_NAME = "/dev/ttyACM0";
static const std::string DEFAULT_TOPIC_ODOM_OUT = "/rookie/odom";

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


struct MessageData
{
    std::string prefix;
    std::string body;
    rclcpp::Time stamp;
    
    static std::optional<MessageData> 
    splitMessage(std::string_view message, const rclcpp::Time& stamp)
    {
        size_t posEndOfPrefix = message.find_first_of(pico_interface::DELIM);
        if (posEndOfPrefix == std::string::npos)
        {
            // std::stringstream ss;
            // ss << "Failed to get message prefix: `" << message << "`" << std::endl;
            // RCLCPP_DEBUG_STREAM(this->_node->get_logger(), ss.str());
            return std::nullopt;
        }
        
        // split into prefix + the rest
        std::string prefix(message, 0, posEndOfPrefix);
        std::string body(message, posEndOfPrefix + 1, std::string::npos);
        // rclcpp::Time stamp = this->_node->now();
        
        return MessageData{prefix, body, rclcpp::Time(stamp)};
    };
    
    static std::string
    toString(const MessageData& msg)
    {
        std::stringstream ss;
        ss << "PREFIX: '" << msg.prefix << "', ";
        ss << "BODY: '" << msg.body << "', ";
        ss << "TIMESTAMP: '" << msg.stamp.nanoseconds() << "'" << std::endl;
        
        return ss.str();
    };

    std::string 
    str() const
    {
        std::stringstream ss;
        ss << "PREFIX: '" << this->prefix << "', ";
        ss << "BODY: '" << this->body << "', ";
        ss << "TIMESTAMP: '" << this->stamp.nanoseconds() << "'" << std::endl;
        
        return ss.str();
    }
};

using MessageCallback = std::function<bool(const MessageData&)>;


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

    bool onReceiveHeartbeat(const MessageData& msg);

    bool onReceiveWheelVelocities(const MessageData& msg);

protected:
private:

    rclcpp::Node::SharedPtr _node;

    MobilityState state = MobilityState::STANDBY;

    // Map inbound message prefixes (e.g., `$HBT`) to callbacks
    std::unordered_map<std::string, MessageCallback> callbacks = {
        {
                pico_interface::MSG_ID_HEARTBEAT, 
                std::bind(&MobilityDriver::onReceiveHeartbeat, this, std::placeholders::_1)
            },
            {
                pico_interface::MSG_ID_VELOCITY_STATUS,
                std::bind(&MobilityDriver::onReceiveWheelVelocities, this, std::placeholders::_1)
            }
    };

    const int spin_rate;

    // Messages received will be published on this topic.
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pico_out_pub;
    // Command velocities to be decomposed into wheel velocities and sent to the base controller.
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
    // Outputs wheel odometry for sensor fusion.
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;

    SerialPort serial_port;

    rclcpp::TimerBase::SharedPtr heartbeat_timer;
    uint32_t heartbeat_seq = 0;

    float desired_vel_left = 0.0;
    float desired_vel_right = 0.0;

    // State of the other side of this connection; could be encapsulated eventually along
    // with any other information I need to track, but for right now this is fine.
    uint32_t last_heartbeat = 0;
};

} // namespace mobility_driver

#endif // MOBILITY_DRIVER_HPP