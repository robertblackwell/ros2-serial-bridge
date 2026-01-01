add_library(serial STATIC
    serial_link/serial_link.cpp
    serial_link/serial_settings.cpp
    serial_link/parser.cpp
    serial_link/sync_serial_link.cpp
    serial_link/serial_asio.cpp

    serial_link/serial_link.h
    serial_link/serial_settings.h
    serial_link/parser.h
    serial_link/sync_serial_link.h
    serial_link/serial_asio.h
    msgs/msgs.h
    msgs/msg_structs.h
    msgs/msgs.cpp
)
target_link_libraries(serial PUBLIC rbl)
target_link_libraries(serial INTERFACE jsoncons)
target_include_directories(serial PUBLIC ${PKG_INC_DIRS})
