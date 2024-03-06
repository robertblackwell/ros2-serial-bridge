
set(TARGET test_with_pico)
add_executable(${TARGET}
		test_with_pico.cpp
		${FBCONTROL_LIB_DIR}/custom.h
		${FBCONTROL_LIB_DIR}/custom.cpp
		${TESTS_COMMON_DIR}/unittest.h
		${TESTS_COMMON_DIR}/unittest.c
		${BRIDGE_LIB_DIR}/serial_settings.h
		${BRIDGE_LIB_DIR}/serial_settings.cpp
)

target_include_directories(${TARGET}
		PUBLIC
		${}JSON_LIB_DIR
		${TESTS_INCLUDE_DIRS}
)
target_link_libraries(${TARGET} /usr/lib/x86_64-linux-gnu/libserial.so)

add_test(${TARGET} ${TARGET})

