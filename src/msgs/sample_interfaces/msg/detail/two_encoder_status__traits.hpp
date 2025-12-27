// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from sample_interfaces:msg/TwoEncoderStatus.idl
// generated code does not contain a copyright notice

#ifndef SAMPLE_INTERFACES__MSG__DETAIL__TWO_ENCODER_STATUS__TRAITS_HPP_
#define SAMPLE_INTERFACES__MSG__DETAIL__TWO_ENCODER_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "sample_interfaces/msg/detail/two_encoder_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'left'
// Member 'right'
#include "sample_interfaces/msg/detail/encoder_status__traits.hpp"

namespace sample_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const TwoEncoderStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: left
  {
    out << "left: ";
    to_flow_style_yaml(msg.left, out);
    out << ", ";
  }

  // member: right
  {
    out << "right: ";
    to_flow_style_yaml(msg.right, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TwoEncoderStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: left
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "left:\n";
    to_block_style_yaml(msg.left, out, indentation + 2);
  }

  // member: right
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "right:\n";
    to_block_style_yaml(msg.right, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TwoEncoderStatus & msg, bool use_flow_style = false)
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
  const sample_interfaces::msg::TwoEncoderStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  sample_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use sample_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const sample_interfaces::msg::TwoEncoderStatus & msg)
{
  return sample_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<sample_interfaces::msg::TwoEncoderStatus>()
{
  return "sample_interfaces::msg::TwoEncoderStatus";
}

template<>
inline const char * name<sample_interfaces::msg::TwoEncoderStatus>()
{
  return "sample_interfaces/msg/TwoEncoderStatus";
}

template<>
struct has_fixed_size<sample_interfaces::msg::TwoEncoderStatus>
  : std::integral_constant<bool, has_fixed_size<sample_interfaces::msg::EncoderStatus>::value> {};

template<>
struct has_bounded_size<sample_interfaces::msg::TwoEncoderStatus>
  : std::integral_constant<bool, has_bounded_size<sample_interfaces::msg::EncoderStatus>::value> {};

template<>
struct is_message<sample_interfaces::msg::TwoEncoderStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SAMPLE_INTERFACES__MSG__DETAIL__TWO_ENCODER_STATUS__TRAITS_HPP_
