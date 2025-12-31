cmake_minimum_required(VERSION 4.1)
project(cpp_serial_bridge LANGUAGES CXX C  VERSION 0.0.1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-error)
endif()
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(PKG_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(PKG_INC_DIRS ${PKG_DIR}/src ${PKG_DIR}/rbl_cpp ${PKG_DIR}/jsoncons)
message("PKG_DIR    ${PKG_DIR}")
message("PKG_INC_DIRS    ${PKG_INC_DIRS}")
add_subdirectory(rbl_cpp ${CMAKE_CURRENT_SOURCE_DIR}/rbl_cpp/build)
add_subdirectory(src)
#add_subdirectory(jsoncons)
add_subdirectory(tests)

