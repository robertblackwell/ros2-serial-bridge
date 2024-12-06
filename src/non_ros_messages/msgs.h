#ifndef H_ros2_bridge_msgs_h
#define H_ros2_bridge_msgs_h
#include <string>
#include <variant>
#include "msg_struct.h"
#include "iobuffer.h"

#ifdef ROS2_MSGS
using namespace sample_interfaces::msg;
#else
using namespace non_ros_msgs;
#endif
using namespace serial_bridge;
/**
 * See the companion package of ROS2 messages called 'sample_interfaces'
 * to understand what message types we area dealing with.
 * 
 * Serialize and Deserialize Functions
 * ===================================
 * These functions are only intended for use by ros2 nodes running on a Linux host.
 *  
 * Hence, Deserialize functions are only required for those messages that originate in the micro-controller
 * and are received by the host. They are:
 * 
 * -    CmdResponse
 * -    TwoEncoderStatus
 * -    TextMsg
 * 
 * Deserialize functions convert a packet of bytes into a std::variant 
 * that holds one of a selection of ROS2 messasge structures.
 * 
 * This variant structure is called an InputMessage
 */
typedef std::variant<TextMsg, CmdResponse, EncoderStatus, TwoEncoderStatus> InputMessage;
typedef std::unique_ptr<InputMessage> InputMessage_Uptr;

/** 
 * The following 4 type specific deserialize functions are only exposed for testing and experimenting
 * and should not be avoided. 
*/
bool deserialize(IoBuffer& buffer, CmdResponse& response);
bool deserialize(IoBuffer& buffer, EncoderStatus& cmd_response);
bool deserialize(IoBuffer& buffer, TwoEncoderStatus& cmd_response);
bool deserialize(IoBuffer& buffer, TextMsg& cmd_response);

/**
 * Use this general deserialize function 
*/
bool deserialize(IoBuffer& buffer, InputMessage& inmsg);

/**
 *  * Similarly serialize functions are only required for those messages sent from the Host to the 
 * Micro-controller. They are:
 * 
 * -    EchoCmd
 * -    LoadTestCmd
 * -    MotorPwmCmd
 * -    MotorRpmCmd
 * -    TextMsgs - I anticipate using these for logging 
 * 
 * Serialize functions convert these ROS2 messages into a packet of bytes ready
 * for transmission to the microcontroller.
 * 
 * The input to the serialize process is again a std::variant which holds instances
 * of one of the ROS2 message types listed above.
 * 
 * The std::variant is called an OutputMessage
*/
typedef std::variant< TextMsg, EchoCmd, LoadTestCmd, MotorPwmCmd, MotorRpmCmd, ReadEncodersCmd> OutputMessage;
typedef std::unique_ptr<OutputMessage> OutputMessage_Uptr;

/** 
 * The following 4 type specific serialize functions are only exposed for testing and experimenting
 * and should not be avoided. 
*/
void serialize(EchoCmd& cmd, IoBuffer& buffer);
void serialize(LoadTestCmd& cmd, IoBuffer& buffer);
void serialize(MotorPwmCmd& cmd, IoBuffer& buffer);
void serialize(MotorRpmCmd& cmd, IoBuffer& buffer);
void serialize(ReadEncodersCmd& cmd_response, IoBuffer& buffer);

/**
 * Use this general serialize function 
*/
void serialize(OutputMessage& out_msg, IoBuffer& buffer);


#endif