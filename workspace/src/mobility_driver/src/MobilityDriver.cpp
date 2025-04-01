// CPP headers
#include <chrono>
using namespace std::chrono_literals;
#include <optional>

// ROS headers
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>

// Rookie-specific
#include <mobility_driver/SerialPort.hpp>
#include <mobility_driver/MobilityDriver.hpp>


namespace mobility_driver
{


MobilityDriver::MobilityDriver(
    const std::string& node_name,
    const std::string& output_topic,
    const int& spin_rate,
    const std::string& serial_device_name)
    :
    spin_rate(spin_rate),
    serial_port(SerialPort(std::string_view(serial_device_name), spin_rate))
{
    this->_node = rclcpp::Node::make_shared(node_name);

    // We'll publish all(-ish) messages from the pico on this topic
    this->pico_out_pub = this->_node->create_publisher<std_msgs::msg::String>(
        output_topic, 
        10
    );

    // Subscribe to /cmd_vel, velocities will be transformed and passed to the pico
    this->vel_sub = this->_node->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 
        10, // TODO: This is the QOS parameter, but 10 is the queue_size?... Not finding a good doc on how this should be used in ROS2...
        std::bind(&MobilityDriver::cmdVelCallback, this, std::placeholders::_1)
    );

    // Send heartbeat messages at 1Hz
    this->heartbeat_timer = this->_node->create_wall_timer(
        1s, 
        std::bind(&MobilityDriver::pushHeartbeat, this));
}

void
MobilityDriver::run()
{
    this->serial_port.start_jthread();

    rclcpp::Rate loop_rate(this->spin_rate);

    std::optional<std::string> inbound;
    while (rclcpp::ok()) 
    {   
        // Get inbound messages from the serial port
        inbound = this->serial_port.pop();
        if (inbound)
        {
            RCLCPP_INFO_STREAM(this->_node->get_logger(), "RX: " << *inbound);
            std::optional<MessageData> msg = MessageData::splitMessage(
                std::string_view(*inbound),
                this->_node->now()
            );
            
            if (msg)
            {
                // TODO:
                // Here's where the ugliness starts; how do I map prefixes to message types to be
                // constructed? The way that PicoInterface is implemented right now doesn't 
                // make this simple. Some options to explore:
                // Option 1) make static functions inside the message structs that return an instance
                //      of that message.
                // Option 2) do some sort of template nonsense?
                // Option 3) skip all of this struct malarkey, go with a base Message() class
                //      and make all of the messages subclasses of that. Then I can just build a factory...
                // Option 4) stop wasting time on this, and just do it the hard-coded way...
                // Option 5) Could I map names of message struct fields to offsets and do some
                //      jank homebrew reflection-esque strangeness? (Pretty sure that's just protobuf-ish shenanigans at this point)
                std::cout << (*msg).str() << std::endl;

                if (auto cb = this->callbacks.find((*msg).prefix); cb != this->callbacks.end())
                {
                    // Execute callbuck function
                    bool status = cb->second(*msg);
                    if (!status) { std::cout << "Failed when processing message: " << *inbound << std::endl; }
                }
            }
        }

        loop_rate.sleep();

        rclcpp::spin_some(this->_node);
    }

    this->serial_port.stop_jthread();
}

void
MobilityDriver::pushHeartbeat()
{
    pico_interface::Msg_Heartbeat heartbeat;
    heartbeat.seq = this->heartbeat_seq++;

    std::string msg;
    pico_interface::message_error_t result = pico_interface::pack_HeartbeatMessage(
        heartbeat, msg);
    if (result != pico_interface::MESSAGE_ERR::E_MSG_SUCCESS) 
    {
        RCLCPP_ERROR_STREAM(this->_node->get_logger(), "HEARTBEAT");
        return;
    }

    this->serial_port.push(std::string_view(msg));
}

void
MobilityDriver::pushVelocity()
{
    pico_interface::Msg_Velocity vel;
    vel.motor_1_velocity = this->desired_vel_left;
    vel.motor_2_velocity = this->desired_vel_right;

    std::string output;
    pico_interface::message_error_t result = pico_interface::pack_Velocity(
        vel, pico_interface::MSG_ID_VELOCITY_CMD, output);
    if (result != pico_interface::MESSAGE_ERR::E_MSG_SUCCESS) 
    {
        RCLCPP_ERROR_STREAM(this->_node->get_logger(), 
            pico_interface::MESSAGE_GET_ERROR(result));
        return;
    }

    this->serial_port.push(std::string_view(output));
}

void 
MobilityDriver::cmdVelCallback(
    geometry_msgs::msg::Twist::UniquePtr msg)
{
    // Do the whole unicycle-model -> diff. drive song and dance number.
    // In the future, I can break this out into its own library
    float linear_vel = msg->linear.x;
    float angular_vel = msg->angular.z;

    // TODO [WF]: I don't know these values yet, but I probably should.
    // Go with best guess for right now...
    float wheelbase = 0.15; // A.K.A. radius of rotation, A.K.A. 'L', in meters
    float wheel_radius = 98.425 / 1000.0 ; // 3-7/8" --> mm --> meters

    float linear_left = linear_vel - ((angular_vel * wheelbase) / 2.0);
    float linear_right = linear_vel + ((angular_vel * wheelbase) / 2.0);

    float angular_left = linear_left / wheel_radius;
    float angular_right = linear_right / wheel_radius;

    this->desired_vel_left = angular_left;
    this->desired_vel_right = angular_right;
}

bool
MobilityDriver::onReceiveHeartbeat(
    const MessageData& msg)
{
    RCLCPP_DEBUG_STREAM(
        this->_node->get_logger(), 
        "GOT A HEARTBEAT: `" << MessageData::toString(msg) << "'"
    );

    // Unpack message
    pico_interface::Msg_Heartbeat heartbeat;
    pico_interface::message_error_t result = pico_interface::unpack_Heartbeat(msg.body, heartbeat);
    if (result != pico_interface::MESSAGE_ERR::E_MSG_SUCCESS) 
    {
        RCLCPP_ERROR_STREAM(this->_node->get_logger(), 
            pico_interface::MESSAGE_GET_ERROR(result));
        return false;
    }

    if (heartbeat.seq < this->last_heartbeat) 
    {
        // TODO: For now, do nothing. in the future, we may want to actually handle this situation,
        // maybe by invalidating our state and starting over? In practice, I think that the 
        // serial connection will lose its mind well before we get to this point.
        RCLCPP_WARN_STREAM(this->_node->get_logger(), 
            "Got heartbeat from the past: " 
            << heartbeat.seq 
            << ", the other end of the connection may have been reset.");    
    }
    this->last_heartbeat = heartbeat.seq;

    return true;
}

} // namespace mobility_driver
