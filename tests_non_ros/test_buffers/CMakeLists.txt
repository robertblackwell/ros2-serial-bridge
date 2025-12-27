# CHECK WE HAVE ALL THE VARIABLES WE NEED
#cmake_minimum_required(VERSION 3.27)
#project(serialio)

set(CMAKE_CXX_STANDARD 20)
set( CMAKE_VERBOSE_MAKEFILE on )

set(TARGET test_buffers)
add_executable(${TARGET}
		test_buffer_main.cpp
	${PKG_DIR}/rbl_cpp/src/iobuffer.h 
	${PKG_DIR}/rbl_cpp/src/iobuffer.cpp
		${PKG_DIR}/rbl_cpp/src/unittest.h
		${PKG_DIR}/rbl_cpp/src/unittest.c
#		../../src/fbcontrol/value_with_direction.h
#		../../src/fbcontrol/motor_profile.cpp
)

target_include_directories(${TARGET}
		PUBLIC
		${TESTS_INCLUDE_DIRS}
)

add_test(${TARGET} ${TARGET})


