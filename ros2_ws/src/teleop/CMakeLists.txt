cmake_minimum_required(VERSION 3.8)
project(teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
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
# Gamepad Command Node
# -------------------------
add_executable(command src/command.cpp)

target_link_libraries(command
  -lquanser_devices
  -lquanser_communications
  -lquanser_runtime
  -lquanser_common
)

ament_target_dependencies(command
  rclcpp
  ament_index_cpp
  geometry_msgs
  std_msgs
)

# -------------------------
# Install
# -------------------------
install(TARGETS
  qbot_platform_driver_interface
  command
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY rt_models
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  AwaitingPickup.mp3
  GoHome.mp3
  DESTINATION share/${PROJECT_NAME}/audio
)

ament_package()
