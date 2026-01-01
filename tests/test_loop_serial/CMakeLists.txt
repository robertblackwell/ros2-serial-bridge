# CHECK WE HAVE ALL THE VARIABLES WE NEED
#cmake_minimum_required(VERSION 3.27)
#project(serialio)


function(test_loop_serial)
set(TARGET test_loop_serial)
add_executable(${TARGET} test_loop_serial.cpp)
target_link_libraries(${TARGET} rbl serial)
add_test(${TARGET} ${TARGET})
endfunction()


#function(test_serial_read)
#set(TARGET test_serial_read)
#add_executable(${TARGET} test_serial_read.cpp)
#target_link_libraries(${TARGET} rbl serial)
#target_compile_definitions(${TARGET} PUBLIC RBL_LOG_ENABLED RBL_LOG_ALLOW_GLOBAL)
#add_test(${TARGET} ${TARGET})
#endfunction()

function(test_serial_read)
set(TARGET test_serial_read)
add_executable(${TARGET} test_serial_read.cpp)
target_link_libraries(${TARGET} rbl serial)
target_compile_definitions(${TARGET} PUBLIC RBL_LOG_ENABLED RBL_LOG_ALLOW_GLOBAL)
add_test(${TARGET} ${TARGET})
endfunction()

function(test_serial_asio_read)
set(TARGET test_serial_asio_read)
add_executable(${TARGET} test_serial_asio_read.cpp)
target_link_libraries(${TARGET} rbl serial)
target_compile_definitions(${TARGET} PUBLIC RBL_LOG_ENABLED RBL_LOG_ALLOW_GLOBAL)
add_test(${TARGET} ${TARGET})
endfunction()

test_loop_serial()
test_serial_read()
test_serial_asio_read()
