// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from sample_interfaces:msg/EncoderStatus.idl
// generated code does not contain a copyright notice

#ifndef SAMPLE_INTERFACES__MSG__DETAIL__ENCODER_STATUS__TRAITS_HPP_
#define SAMPLE_INTERFACES__MSG__DETAIL__ENCODER_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "sample_interfaces/msg/detail/encoder_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace sample_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const EncoderStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: sample_sum
  {
    out << "sample_sum: ";
    rosidl_generator_traits::value_to_yaml(msg.sample_sum, out);
    out << ", ";
  }

  // member: sample_time_stamp_usecs
  {
    out << "sample_time_stamp_usecs: ";
    rosidl_generator_traits::value_to_yaml(msg.sample_time_stamp_usecs, out);
    out << ", ";
  }

  // member: motor_rpm_estimate
  {
    out << "motor_rpm_estimate: ";
    rosidl_generator_traits::value_to_yaml(msg.motor_rpm_estimate, out);
    out << ", ";
  }

  // member: direction_pin_state
  {
    out << "direction_pin_state: ";
    rosidl_generator_traits::value_to_yaml(msg.direction_pin_state, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const EncoderStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: sample_sum
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sample_sum: ";
    rosidl_generator_traits::value_to_yaml(msg.sample_sum, out);
    out << "\n";
  }

  // member: sample_time_stamp_usecs
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sample_time_stamp_usecs: ";
    rosidl_generator_traits::value_to_yaml(msg.sample_time_stamp_usecs, out);
    out << "\n";
  }

  // member: motor_rpm_estimate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "motor_rpm_estimate: ";
    rosidl_generator_traits::value_to_yaml(msg.motor_rpm_estimate, out);
    out << "\n";
  }

  // member: direction_pin_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "direction_pin_state: ";
    rosidl_generator_traits::value_to_yaml(msg.direction_pin_state, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const EncoderStatus & msg, bool use_flow_style = false)
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
  const sample_interfaces::msg::EncoderStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  sample_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use sample_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const sample_interfaces::msg::EncoderStatus & msg)
{
  return sample_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<sample_interfaces::msg::EncoderStatus>()
{
  return "sample_interfaces::msg::EncoderStatus";
}

template<>
inline const char * name<sample_interfaces::msg::EncoderStatus>()
{
  return "sample_interfaces/msg/EncoderStatus";
}

template<>
struct has_fixed_size<sample_interfaces::msg::EncoderStatus>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<sample_interfaces::msg::EncoderStatus>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<sample_interfaces::msg::EncoderStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SAMPLE_INTERFACES__MSG__DETAIL__ENCODER_STATUS__TRAITS_HPP_
