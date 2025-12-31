D="${HOME}/Projects/ros2-serial-bridge/cmake-build-debug"
echo ${D}
F1="${D}/tests/test_buffers/CMakeFiles/test_buffers.dir/test_buffer_main.cpp.o"
F2="${D}/tests/test_buffers/CMakeFiles/test_buffers.dir/__/__/rbl_cpp/rbl/iobuffer.cpp.o"
F3="${D}/tests/test_buffers/CMakeFiles/test_buffers.dir/__/__/rbl_cpp/rbl/unittest.cpp.o"
F4="${D}/tests/test_buffers/test_buffers"
echo $F1
echo $F2
echo $F3
echo $F4
[ -f "$F1" ] && echo "F1 File found" || echo "F1 File not found"
[ -f "$F2" ] && echo "F2 File found" || echo "F2 File not found"
[ -f "$F3" ] && echo "F3 File found" || echo "F3 File not found"
[ -f "$F4" ] && echo "F4 File found" || echo "F4 File not found"

/usr/bin/c++ -frtti -fvisibility-inlines-hidden -fvisibility=hidden -pthread -g -Wl, ${F1} ${F2} ${F3} -o ${F4}


#    ${D}/tests/test_buffers/CMakeFiles/test_buffers.dir/test_buffer_main.cpp.o \
#    ${D}/tests/test_buffers/CMakeFiles/test_buffers.dir/__/__/rbl_cpp/rbl/iobuffer.cpp.o \
#    ${D}/tests/test_buffers/CMakeFiles/test_buffers.dir/__/__/rbl_cpp/rbl/unittest.cpp.o \
