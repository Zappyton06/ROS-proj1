cmake_minimum_required(VERSION 3.8)
project(xy_generator)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/XY.msg"
)

# Include directories
include_directories(
  include
)

# Declare and build the publisher node executable
add_executable(xy_publisher src/xy_publisher.cpp)
ament_target_dependencies(xy_publisher rclcpp std_msgs)

# Declare and build the subscriber node executable
add_executable(xy_subscriber src/xy_subscriber.cpp)
ament_target_dependencies(xy_subscriber rclcpp std_msgs)

# Link message generation dependencies
rosidl_target_interfaces(xy_publisher ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(xy_subscriber ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install the executables
install(TARGETS
  xy_publisher
  xy_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)
ament_package()
