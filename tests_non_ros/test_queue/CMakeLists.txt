# CHECK WE HAVE ALL THE VARIABLES WE NEED
#cmake_minimum_required(VERSION 3.27)
#project(serialio)

set(CMAKE_CXX_STANDARD 20)
set( CMAKE_VERBOSE_MAKEFILE on )

set(TARGET test_queue)
add_executable(${TARGET}
        test_queue_main.cpp
		${PKG_DIR}/rbl_cpp/rbl/queue.h
		${PKG_DIR}/rbl_cpp/rbl/fd_functions.cpp
		${PKG_DIR}/rbl_cpp/rbl/unittest.h
		${PKG_DIR}/rbl_cpp/rbl/unittest.c
		)

target_include_directories(${TARGET}
		PUBLIC
		${TESTS_INCLUDE_DIRS}
)


add_test(${TARGET} ${TARGET})


