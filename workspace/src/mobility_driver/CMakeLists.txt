cmake_minimum_required(VERSION 3.8)
project(mobility_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Werror -Wextra -Wpedantic)
    set(CMAKE_CXX_STANDARD 20)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS "ON")

# find ROS dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# find OTHER dependencies
find_package(pico_interface 0.0.1 REQUIRED)


#########################
## Libraries
#########################

add_library(mobility_driver_lib
    src/SerialPort.cpp
    src/MobilityDriver.cpp
)
target_include_directories(mobility_driver_lib
    PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
        ${PICO_INTERFACE_INCLUDE_DIRS}
)
ament_target_dependencies(mobility_driver_lib
    PUBLIC
        rclcpp
        std_msgs
        geometry_msgs
)

########################
## Executables
########################

add_executable(mobility_driver_node
    src/mobility_driver_node.cpp
)
target_link_libraries(mobility_driver_node
    mobility_driver_lib
)
ament_target_dependencies(mobility_driver_node
    rclcpp
    std_msgs
    geometry_msgs
)

########################
## Install
########################

# Libraries
install(
    TARGETS mobility_driver_lib
    EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

# Headers
install(
    DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
)

# Executables
install(
    TARGETS
        mobility_driver_node
    DESTINATION
        lib/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
    rclcpp 
    std_msgs
)

#######################
## Testing
#######################

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
