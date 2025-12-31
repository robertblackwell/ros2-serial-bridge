set(TARGET test_buffers)
add_executable(${TARGET} test_buffer_main.cpp)
target_link_libraries(${TARGET} PUBLIC rbl serial)
add_test(${TARGET} ${TARGET})


