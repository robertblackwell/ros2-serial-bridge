// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sample_interfaces:msg/ReadEncodersCmd.idl
// generated code does not contain a copyright notice

#ifndef SAMPLE_INTERFACES__MSG__DETAIL__READ_ENCODERS_CMD__STRUCT_HPP_
#define SAMPLE_INTERFACES__MSG__DETAIL__READ_ENCODERS_CMD__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__sample_interfaces__msg__ReadEncodersCmd __attribute__((deprecated))
#else
# define DEPRECATED__sample_interfaces__msg__ReadEncodersCmd __declspec(deprecated)
#endif

namespace sample_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ReadEncodersCmd_
{
  using Type = ReadEncodersCmd_<ContainerAllocator>;

  explicit ReadEncodersCmd_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->n = 0l;
    }
  }

  explicit ReadEncodersCmd_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->n = 0l;
    }
  }

  // field types and members
  using _n_type =
    int32_t;
  _n_type n;

  // setters for named parameter idiom
  Type & set__n(
    const int32_t & _arg)
  {
    this->n = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    sample_interfaces::msg::ReadEncodersCmd_<ContainerAllocator> *;
  using ConstRawPtr =
    const sample_interfaces::msg::ReadEncodersCmd_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sample_interfaces::msg::ReadEncodersCmd_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sample_interfaces::msg::ReadEncodersCmd_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sample_interfaces::msg::ReadEncodersCmd_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sample_interfaces::msg::ReadEncodersCmd_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sample_interfaces::msg::ReadEncodersCmd_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sample_interfaces::msg::ReadEncodersCmd_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sample_interfaces::msg::ReadEncodersCmd_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sample_interfaces::msg::ReadEncodersCmd_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sample_interfaces__msg__ReadEncodersCmd
    std::shared_ptr<sample_interfaces::msg::ReadEncodersCmd_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sample_interfaces__msg__ReadEncodersCmd
    std::shared_ptr<sample_interfaces::msg::ReadEncodersCmd_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ReadEncodersCmd_ & other) const
  {
    if (this->n != other.n) {
      return false;
    }
    return true;
  }
  bool operator!=(const ReadEncodersCmd_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ReadEncodersCmd_

// alias to use template instance with default allocator
using ReadEncodersCmd =
  sample_interfaces::msg::ReadEncodersCmd_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace sample_interfaces

#endif  // SAMPLE_INTERFACES__MSG__DETAIL__READ_ENCODERS_CMD__STRUCT_HPP_
