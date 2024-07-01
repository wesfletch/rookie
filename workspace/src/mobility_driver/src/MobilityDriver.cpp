
// ROS msgs
#include <std_msgs/msg/string.hpp>


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

void
MobilityDriver::run()
{
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