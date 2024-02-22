cmake_minimum_required(VERSION 3.8)
project(cpp_serial_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 20)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(
  bridge src/bridge.cpp
  src/bridge_lib/queue.h
  src/bridge_lib/ros2_bridge.h
  src/bridge_lib/ros2_bridge.cpp
  src/bridge_lib/serial_settings.h
  src/bridge_lib/serial_settings.cpp
  src/bridge_lib/iobuffer.h
  src/bridge_lib/iobuffer.cpp
  src/bridge_lib/fd_functions.h
  src/bridge_lib/fd_functions.cpp
  src/bridge_lib/msgs.h
  src/bridge_lib/msgs.cpp

  ) 
ament_target_dependencies(bridge rclcpp std_msgs)

install(TARGETS 
  bridge
  DESTINATION lib/${PROJECT_NAME})

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
