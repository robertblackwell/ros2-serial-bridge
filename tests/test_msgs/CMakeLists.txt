set(TARGET test_msgs)
add_executable(${TARGET} test_msgs.cpp)
target_link_libraries(${TARGET} rbl)
target_link_libraries(${TARGET} serial)
target_link_libraries(${TARGET} jsoncons)
target_compile_definitions(${TARGET}
	PUBLIC 
		LOCAL_TEST=1 
		PLATFORM_IS_LINUX=1, 
		TRANSPORT_MODE_PACKET=0
)
add_test(${TARGET} ${TARGET})

