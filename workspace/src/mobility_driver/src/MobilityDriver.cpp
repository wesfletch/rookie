
// C headers
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

// CPP headers
#include <chrono>
// I HATE this
using namespace std::chrono_literals;

#include <optional>

// ROS headers
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>

// Rookie-specific
#include <mobility_driver/MobilityDriver.hpp>
#include <pico_interface/PicoInterface.hpp>


namespace mobility_driver
{


MobilityDriver::MobilityDriver()
{
    this->_node = rclcpp::Node::make_shared("mobility_driver");

    if (!this->serial_port.configure(100)) 
    { 
        throw std::runtime_error("Failed to configure serial port.");  
    }

    // We'll publish all(-ish) messages from the pico on this topic
    this->pico_out_pub = this->_node->create_publisher<std_msgs::msg::String>(
        "pico_out", 10
    );

    // Subscribe to /cmd_vel, velocities will be transformed and passed to the pico
    this->vel_sub = this->_node->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 
        10, 
        std::bind(&MobilityDriver::cmdVelCallback, this, std::placeholders::_1));

    // Configure a timer to send heartbeat messages
    this->heartbeat_timer = this->_node->create_wall_timer(
        1s, 
        std::bind(&MobilityDriver::pushHeartbeat, this));

    
}

void
MobilityDriver::run()
{
    int x = 100;

    rclcpp::Rate loop_rate(100); // roughly 2x the spin rate of the pico to avoid buffer-filling
    while (rclcpp::ok()) 
    {   
        std::optional<std::string> out = this->serial_port.spinOnce();
        if (out) 
        {
            RCLCPP_INFO_STREAM(this->_node->get_logger(), "RX: " << *out);
        }

        if ((++x % 100) == 0)
        {
            this->pushVelocity();
        }

        loop_rate.sleep();

        rclcpp::spin_some(this->_node);
    }
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
        RCLCPP_ERROR_STREAM(this->_node->get_logger(), "HEARTBEAT FUCK");
        return;
    }

    {
        std::lock_guard<std::mutex> lock(this->serial_port_mtx);
        this->serial_port.write(msg);
    }
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

    {
        std::lock_guard<std::mutex> lock(this->serial_port_mtx);
        this->serial_port.write(output);
    }
    
}

void 
MobilityDriver::cmdVelCallback(
    [[maybe_unused]] geometry_msgs::msg::Twist::UniquePtr msg)
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

} // namespace mobility_driver
