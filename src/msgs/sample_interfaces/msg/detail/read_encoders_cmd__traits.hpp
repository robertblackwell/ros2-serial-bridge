// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from sample_interfaces:msg/ReadEncodersCmd.idl
// generated code does not contain a copyright notice

#ifndef SAMPLE_INTERFACES__MSG__DETAIL__READ_ENCODERS_CMD__TRAITS_HPP_
#define SAMPLE_INTERFACES__MSG__DETAIL__READ_ENCODERS_CMD__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "sample_interfaces/msg/detail/read_encoders_cmd__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace sample_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const ReadEncodersCmd & msg,
  std::ostream & out)
{
  out << "{";
  // member: n
  {
    out << "n: ";
    rosidl_generator_traits::value_to_yaml(msg.n, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ReadEncodersCmd & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: n
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "n: ";
    rosidl_generator_traits::value_to_yaml(msg.n, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ReadEncodersCmd & msg, bool use_flow_style = false)
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
  const sample_interfaces::msg::ReadEncodersCmd & msg,
  std::ostream & out, size_t indentation = 0)
{
  sample_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use sample_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const sample_interfaces::msg::ReadEncodersCmd & msg)
{
  return sample_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<sample_interfaces::msg::ReadEncodersCmd>()
{
  return "sample_interfaces::msg::ReadEncodersCmd";
}

template<>
inline const char * name<sample_interfaces::msg::ReadEncodersCmd>()
{
  return "sample_interfaces/msg/ReadEncodersCmd";
}

template<>
struct has_fixed_size<sample_interfaces::msg::ReadEncodersCmd>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<sample_interfaces::msg::ReadEncodersCmd>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<sample_interfaces::msg::ReadEncodersCmd>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SAMPLE_INTERFACES__MSG__DETAIL__READ_ENCODERS_CMD__TRAITS_HPP_
