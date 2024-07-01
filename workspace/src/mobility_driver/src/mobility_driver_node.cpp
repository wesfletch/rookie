#include <rclcpp/rclcpp.hpp>

#include <mobility_driver/MobilityDriver.hpp>

int 
main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    
    auto mobility_driver = std::make_shared<mobility_driver::MobilityDriver>();
    
    // rclcpp::spin();
    mobility_driver->run();
    rclcpp::shutdown();
    
    return 0;
}