add_library(serial STATIC)
target_include_directories(serial PUBLIC ${PKG_INC_DIRS})
target_sources(serial PUBLIC
    serial_link/serial_link.cpp
    serial_link/serial_settings.cpp
    serial_link/parser.cpp
    serial_link/sync_serial_link.cpp

    serial_link/serial_link.h
    serial_link/serial_settings.h
    serial_link/parser.h
    serial_link/sync_serial_link.h
)
#add_subdirectory(serial_link)
get_target_property(inc01 serial INCLUDE_DIRECTORIES)

message("serial lib include dirs  ${inc01}" )
message("serial lib include dirs  ${PKG_INC_DIRS}" )