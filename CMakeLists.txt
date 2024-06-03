cmake_minimum_required(VERSION 3.5)
project(yesense_imu)

## Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        ./src
)

## Declare a C++ executable
add_executable(yesense_imu_node src/yesense_imu_node.cpp src/analysis_data.c)

## Specify libraries to link a library or executable target against
ament_target_dependencies(yesense_imu_node
        rclcpp
        geometry_msgs
        sensor_msgs
        std_msgs
        tf2
        tf2_ros
)

add_executable(example src/example.c src/analysis_data.c)
ament_target_dependencies(example
        rclcpp
        geometry_msgs
        sensor_msgs
        std_msgs
        tf2
        tf2_ros
)

## Install executables and libraries
install(TARGETS
        yesense_imu_node
        example
        DESTINATION lib/${PROJECT_NAME}
)

## Install other files like launch files and rviz configurations
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY rviz
        DESTINATION share/${PROJECT_NAME}
)

## Export package
ament_package()
