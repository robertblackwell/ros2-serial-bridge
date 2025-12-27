// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from sample_interfaces:msg/EchoCmd.idl
// generated code does not contain a copyright notice

#ifndef SAMPLE_INTERFACES__MSG__DETAIL__ECHO_CMD__TRAITS_HPP_
#define SAMPLE_INTERFACES__MSG__DETAIL__ECHO_CMD__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "sample_interfaces/msg/detail/echo_cmd__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace sample_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const EchoCmd & msg,
  std::ostream & out)
{
  out << "{";
  // member: data
  {
    if (msg.data.size() == 0) {
      out << "data: []";
    } else {
      out << "data: [";
      size_t pending_items = msg.data.size();
      for (auto item : msg.data) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const EchoCmd & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.data.size() == 0) {
      out << "data: []\n";
    } else {
      out << "data:\n";
      for (auto item : msg.data) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const EchoCmd & msg, bool use_flow_style = false)
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
  const sample_interfaces::msg::EchoCmd & msg,
  std::ostream & out, size_t indentation = 0)
{
  sample_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use sample_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const sample_interfaces::msg::EchoCmd & msg)
{
  return sample_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<sample_interfaces::msg::EchoCmd>()
{
  return "sample_interfaces::msg::EchoCmd";
}

template<>
inline const char * name<sample_interfaces::msg::EchoCmd>()
{
  return "sample_interfaces/msg/EchoCmd";
}

template<>
struct has_fixed_size<sample_interfaces::msg::EchoCmd>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<sample_interfaces::msg::EchoCmd>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<sample_interfaces::msg::EchoCmd>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SAMPLE_INTERFACES__MSG__DETAIL__ECHO_CMD__TRAITS_HPP_
