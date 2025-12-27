// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sample_interfaces:msg/LoadTestCmd.idl
// generated code does not contain a copyright notice

#ifndef SAMPLE_INTERFACES__MSG__DETAIL__LOAD_TEST_CMD__STRUCT_HPP_
#define SAMPLE_INTERFACES__MSG__DETAIL__LOAD_TEST_CMD__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__sample_interfaces__msg__LoadTestCmd __attribute__((deprecated))
#else
# define DEPRECATED__sample_interfaces__msg__LoadTestCmd __declspec(deprecated)
#endif

namespace sample_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LoadTestCmd_
{
  using Type = LoadTestCmd_<ContainerAllocator>;

  explicit LoadTestCmd_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->count = 0l;
      this->msg_length = 0l;
      this->msgs_per_seecond = 0l;
    }
  }

  explicit LoadTestCmd_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->count = 0l;
      this->msg_length = 0l;
      this->msgs_per_seecond = 0l;
    }
  }

  // field types and members
  using _count_type =
    int32_t;
  _count_type count;
  using _msg_length_type =
    int32_t;
  _msg_length_type msg_length;
  using _msgs_per_seecond_type =
    int32_t;
  _msgs_per_seecond_type msgs_per_seecond;

  // setters for named parameter idiom
  Type & set__count(
    const int32_t & _arg)
  {
    this->count = _arg;
    return *this;
  }
  Type & set__msg_length(
    const int32_t & _arg)
  {
    this->msg_length = _arg;
    return *this;
  }
  Type & set__msgs_per_seecond(
    const int32_t & _arg)
  {
    this->msgs_per_seecond = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    sample_interfaces::msg::LoadTestCmd_<ContainerAllocator> *;
  using ConstRawPtr =
    const sample_interfaces::msg::LoadTestCmd_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sample_interfaces::msg::LoadTestCmd_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sample_interfaces::msg::LoadTestCmd_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sample_interfaces::msg::LoadTestCmd_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sample_interfaces::msg::LoadTestCmd_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sample_interfaces::msg::LoadTestCmd_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sample_interfaces::msg::LoadTestCmd_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sample_interfaces::msg::LoadTestCmd_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sample_interfaces::msg::LoadTestCmd_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sample_interfaces__msg__LoadTestCmd
    std::shared_ptr<sample_interfaces::msg::LoadTestCmd_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sample_interfaces__msg__LoadTestCmd
    std::shared_ptr<sample_interfaces::msg::LoadTestCmd_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LoadTestCmd_ & other) const
  {
    if (this->count != other.count) {
      return false;
    }
    if (this->msg_length != other.msg_length) {
      return false;
    }
    if (this->msgs_per_seecond != other.msgs_per_seecond) {
      return false;
    }
    return true;
  }
  bool operator!=(const LoadTestCmd_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LoadTestCmd_

// alias to use template instance with default allocator
using LoadTestCmd =
  sample_interfaces::msg::LoadTestCmd_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace sample_interfaces

#endif  // SAMPLE_INTERFACES__MSG__DETAIL__LOAD_TEST_CMD__STRUCT_HPP_
