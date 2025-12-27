// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from sample_interfaces:msg/MotorRpmCmd.idl
// generated code does not contain a copyright notice

#ifndef SAMPLE_INTERFACES__MSG__DETAIL__MOTOR_RPM_CMD__TRAITS_HPP_
#define SAMPLE_INTERFACES__MSG__DETAIL__MOTOR_RPM_CMD__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "sample_interfaces/msg/detail/motor_rpm_cmd__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace sample_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const MotorRpmCmd & msg,
  std::ostream & out)
{
  out << "{";
  // member: left_motor
  {
    out << "left_motor: ";
    rosidl_generator_traits::value_to_yaml(msg.left_motor, out);
    out << ", ";
  }

  // member: right_motor
  {
    out << "right_motor: ";
    rosidl_generator_traits::value_to_yaml(msg.right_motor, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MotorRpmCmd & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: left_motor
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "left_motor: ";
    rosidl_generator_traits::value_to_yaml(msg.left_motor, out);
    out << "\n";
  }

  // member: right_motor
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "right_motor: ";
    rosidl_generator_traits::value_to_yaml(msg.right_motor, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MotorRpmCmd & msg, bool use_flow_style = false)
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
  const sample_interfaces::msg::MotorRpmCmd & msg,
  std::ostream & out, size_t indentation = 0)
{
  sample_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use sample_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const sample_interfaces::msg::MotorRpmCmd & msg)
{
  return sample_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<sample_interfaces::msg::MotorRpmCmd>()
{
  return "sample_interfaces::msg::MotorRpmCmd";
}

template<>
inline const char * name<sample_interfaces::msg::MotorRpmCmd>()
{
  return "sample_interfaces/msg/MotorRpmCmd";
}

template<>
struct has_fixed_size<sample_interfaces::msg::MotorRpmCmd>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<sample_interfaces::msg::MotorRpmCmd>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<sample_interfaces::msg::MotorRpmCmd>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SAMPLE_INTERFACES__MSG__DETAIL__MOTOR_RPM_CMD__TRAITS_HPP_
