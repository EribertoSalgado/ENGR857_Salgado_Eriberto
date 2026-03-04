cmake_minimum_required(VERSION 3.8)
project(follower)

# Use C++17 for std::clamp and structured bindings
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)


# Required for Quanser SDK
add_compile_definitions(_NVIDIA)

# -------------------------
# Driver Interface
# -------------------------
add_executable(qbot_platform_driver_interface src/qbot_platform_driver_interface.cpp)

target_link_libraries(qbot_platform_driver_interface
  -lquanser_devices
  -lquanser_communications
  -lquanser_runtime
  -lquanser_common
)

ament_target_dependencies(qbot_platform_driver_interface
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  control_msgs
)

# -------------------------
# Follower Node
# -------------------------
add_executable(follower src/command.cpp)

ament_target_dependencies(follower
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
)

# -------------------------
# Install
# -------------------------
install(TARGETS
  qbot_platform_driver_interface
  follower
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY rt_models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
