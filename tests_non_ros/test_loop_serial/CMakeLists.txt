# CHECK WE HAVE ALL THE VARIABLES WE NEED
#cmake_minimum_required(VERSION 3.27)
#project(serialio)

set(CMAKE_CXX_STANDARD 20)
set( CMAKE_VERBOSE_MAKEFILE on )
set(TARGET test_loop_serial)
add_executable(${TARGET}
    test_loop_serial.cpp
    ${PKG_DIR}/src/serial_link/serial_link.cpp
    ${PKG_DIR}/src/serial_link/serial_link.h
    ${PKG_DIR}/src/serial_link/serial_settings.h
    ${PKG_DIR}/src/serial_link/serial_settings.cpp
	${PKG_DIR}/rbl_cpp/src/iobuffer.h
	${PKG_DIR}/rbl_cpp/src/iobuffer.cpp
    ${PKG_DIR}/rbl_cpp/src/unittest.h
    ${PKG_DIR}/rbl_cpp/src/unittest.c
    ${PKG_DIR}/rbl_cpp/src/logger.h
    ${PKG_DIR}/rbl_cpp/src/logger.cpp
)

target_include_directories(${TARGET}
		PUBLIC
		${TESTS_INCLUDE_DIRS}
)

add_test(${TARGET} ${TARGET})


