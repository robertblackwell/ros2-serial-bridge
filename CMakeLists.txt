cmake_minimum_required(VERSION 3.8)
project(cpp_serial_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sample_interfaces REQUIRED)

set(SRC_LIST 
	src/bridge_lib/queue.h
	src/bridge_lib/serial_link.h
	src/bridge_lib/serial_link.cpp
	src/bridge_lib/serial_settings.h
	src/bridge_lib/serial_settings.cpp
	src/bridge_lib/iobuffer.h
	src/bridge_lib/iobuffer.cpp
	src/bridge_lib/fd_functions.h
	src/bridge_lib/fd_functions.cpp
	src/bridge_lib/msgs.h
	src/bridge_lib/msgs.cpp
)

# bridge
add_executable(bridge src/bridge.cpp 	${SRC_LIST})
target_include_directories(bridge PUBLIC 
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${CMAKE_CURRENT_LIST_DIR}/src
)
ament_target_dependencies(bridge rclcpp std_msgs geometry_msgs sample_interfaces)

# bridge_test_cmds
add_executable(bridge_test_cmds src/bridge_test_cmds.cpp ${SRC_LIST})
target_include_directories(bridge_test_cmds PUBLIC 
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${CMAKE_CURRENT_LIST_DIR}/src
)
ament_target_dependencies(bridge_test_cmds rclcpp std_msgs geometry_msgs sample_interfaces)

# bridge_speed_test
add_executable(bridge_speed_test src/bridge_speed_test.cpp ${SRC_LIST})
target_include_directories(bridge_speed_test PUBLIC 
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${CMAKE_CURRENT_LIST_DIR}/src
)
ament_target_dependencies(bridge_speed_test rclcpp std_msgs geometry_msgs sample_interfaces)
target_compile_definitions(bridge_speed_test 
            PUBLIC
            BRIDGE_SPEED_TEST_MESSAGE_COUNT=30000
            BRIDGE_SPEED_TEST_VERBOSE=2
)

# publish_test_cmds
add_executable(publisher_test_cmds src/publisher_test_cmds.cpp ${SRC_LIST})
target_include_directories(publisher_test_cmds PUBLIC 
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${CMAKE_CURRENT_LIST_DIR}/src
)
ament_target_dependencies(publisher_test_cmds rclcpp std_msgs geometry_msgs sample_interfaces)
target_compile_definitions(publisher_test_cmds 
            PUBLIC
            BRIDGE_SPEED_TEST_MESSAGE_COUNT=30000
            BRIDGE_SPEED_TEST_VERBOSE=2
)


install(TARGETS 
	bridge
	bridge_speed_test
	bridge_test_cmds
	publisher_test_cmds
	DESTINATION lib/${PROJECT_NAME})

# if(BUILD_TESTING)
#   find_package(ament_cmake_gtest REQUIRED)
#   ament_add_gtest(${PROJECT_NAME}_test_01 test/test_01.cpp)
#   target_include_directories(${PROJECT_NAME}_test_01 PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:include>
#   )
#   ament_target_dependencies(${PROJECT_NAME}_test_01  std_msgs)
#   # ament_target_link_libraries(${PROJECT_NAME}_test_01)# name_of_local_library)
# endif()

ament_package()
