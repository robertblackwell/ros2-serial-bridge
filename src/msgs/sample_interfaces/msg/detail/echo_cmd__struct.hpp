// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sample_interfaces:msg/EchoCmd.idl
// generated code does not contain a copyright notice

#ifndef SAMPLE_INTERFACES__MSG__DETAIL__ECHO_CMD__STRUCT_HPP_
#define SAMPLE_INTERFACES__MSG__DETAIL__ECHO_CMD__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__sample_interfaces__msg__EchoCmd __attribute__((deprecated))
#else
# define DEPRECATED__sample_interfaces__msg__EchoCmd __declspec(deprecated)
#endif

namespace sample_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct EchoCmd_
{
  using Type = EchoCmd_<ContainerAllocator>;

  explicit EchoCmd_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit EchoCmd_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _data_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__data(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    sample_interfaces::msg::EchoCmd_<ContainerAllocator> *;
  using ConstRawPtr =
    const sample_interfaces::msg::EchoCmd_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sample_interfaces::msg::EchoCmd_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sample_interfaces::msg::EchoCmd_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sample_interfaces::msg::EchoCmd_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sample_interfaces::msg::EchoCmd_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sample_interfaces::msg::EchoCmd_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sample_interfaces::msg::EchoCmd_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sample_interfaces::msg::EchoCmd_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sample_interfaces::msg::EchoCmd_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sample_interfaces__msg__EchoCmd
    std::shared_ptr<sample_interfaces::msg::EchoCmd_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sample_interfaces__msg__EchoCmd
    std::shared_ptr<sample_interfaces::msg::EchoCmd_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const EchoCmd_ & other) const
  {
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const EchoCmd_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct EchoCmd_

// alias to use template instance with default allocator
using EchoCmd =
  sample_interfaces::msg::EchoCmd_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace sample_interfaces

#endif  // SAMPLE_INTERFACES__MSG__DETAIL__ECHO_CMD__STRUCT_HPP_
