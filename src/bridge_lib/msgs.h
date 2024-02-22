#ifndef H_ros2_bridge_msgs_h
#define H_ros2_bridge_msgs_h
#include <string>
#include "iobuffer.h"
typedef std::string Message;

Message deserializer(ros2_bridge::IoBuffer& message_buffer);
void serializer(Message message, ros2_bridge::IoBuffer& iobuffer);




#endif