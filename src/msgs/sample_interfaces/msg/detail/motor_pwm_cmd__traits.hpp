// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from sample_interfaces:msg/MotorPwmCmd.idl
// generated code does not contain a copyright notice

#ifndef SAMPLE_INTERFACES__MSG__DETAIL__MOTOR_PWM_CMD__TRAITS_HPP_
#define SAMPLE_INTERFACES__MSG__DETAIL__MOTOR_PWM_CMD__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "sample_interfaces/msg/detail/motor_pwm_cmd__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace sample_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const MotorPwmCmd & msg,
  std::ostream & out)
{
  out << "{";
  // member: left_motor_pwm
  {
    out << "left_motor_pwm: ";
    rosidl_generator_traits::value_to_yaml(msg.left_motor_pwm, out);
    out << ", ";
  }

  // member: right_motor_pwm
  {
    out << "right_motor_pwm: ";
    rosidl_generator_traits::value_to_yaml(msg.right_motor_pwm, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MotorPwmCmd & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: left_motor_pwm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "left_motor_pwm: ";
    rosidl_generator_traits::value_to_yaml(msg.left_motor_pwm, out);
    out << "\n";
  }

  // member: right_motor_pwm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "right_motor_pwm: ";
    rosidl_generator_traits::value_to_yaml(msg.right_motor_pwm, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MotorPwmCmd & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace sample_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use sample_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const sample_interfaces::msg::MotorPwmCmd & msg,
  std::ostream & out, size_t indentation = 0)
{
  sample_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use sample_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const sample_interfaces::msg::MotorPwmCmd & msg)
{
  return sample_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<sample_interfaces::msg::MotorPwmCmd>()
{
  return "sample_interfaces::msg::MotorPwmCmd";
}

template<>
inline const char * name<sample_interfaces::msg::MotorPwmCmd>()
{
  return "sample_interfaces/msg/MotorPwmCmd";
}

template<>
struct has_fixed_size<sample_interfaces::msg::MotorPwmCmd>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<sample_interfaces::msg::MotorPwmCmd>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<sample_interfaces::msg::MotorPwmCmd>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SAMPLE_INTERFACES__MSG__DETAIL__MOTOR_PWM_CMD__TRAITS_HPP_
