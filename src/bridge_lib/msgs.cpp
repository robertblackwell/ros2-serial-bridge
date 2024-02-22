#include "msgs.h"
#include "iobuffer.h"

Message deserializer(ros2_bridge::IoBuffer& message_buffer)
{
    return message_buffer.to_string();
}

void serializer(Message message, ros2_bridge::IoBuffer& iobuffer)
{
    if(! iobuffer.empty()) {
        throw std::runtime_error("serialize iobuffer should be empty");
    }
    iobuffer.append(message);
}
