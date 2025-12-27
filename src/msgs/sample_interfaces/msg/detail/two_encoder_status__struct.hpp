// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sample_interfaces:msg/TwoEncoderStatus.idl
// generated code does not contain a copyright notice

#ifndef SAMPLE_INTERFACES__MSG__DETAIL__TWO_ENCODER_STATUS__STRUCT_HPP_
#define SAMPLE_INTERFACES__MSG__DETAIL__TWO_ENCODER_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'left'
// Member 'right'
#include "sample_interfaces/msg/detail/encoder_status__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__sample_interfaces__msg__TwoEncoderStatus __attribute__((deprecated))
#else
# define DEPRECATED__sample_interfaces__msg__TwoEncoderStatus __declspec(deprecated)
#endif

namespace sample_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TwoEncoderStatus_
{
  using Type = TwoEncoderStatus_<ContainerAllocator>;

  explicit TwoEncoderStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : left(_init),
    right(_init)
  {
    (void)_init;
  }

  explicit TwoEncoderStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : left(_alloc, _init),
    right(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _left_type =
    sample_interfaces::msg::EncoderStatus_<ContainerAllocator>;
  _left_type left;
  using _right_type =
    sample_interfaces::msg::EncoderStatus_<ContainerAllocator>;
  _right_type right;

  // setters for named parameter idiom
  Type & set__left(
    const sample_interfaces::msg::EncoderStatus_<ContainerAllocator> & _arg)
  {
    this->left = _arg;
    return *this;
  }
  Type & set__right(
    const sample_interfaces::msg::EncoderStatus_<ContainerAllocator> & _arg)
  {
    this->right = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    sample_interfaces::msg::TwoEncoderStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const sample_interfaces::msg::TwoEncoderStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sample_interfaces::msg::TwoEncoderStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sample_interfaces::msg::TwoEncoderStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sample_interfaces::msg::TwoEncoderStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sample_interfaces::msg::TwoEncoderStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sample_interfaces::msg::TwoEncoderStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sample_interfaces::msg::TwoEncoderStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sample_interfaces::msg::TwoEncoderStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sample_interfaces::msg::TwoEncoderStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sample_interfaces__msg__TwoEncoderStatus
    std::shared_ptr<sample_interfaces::msg::TwoEncoderStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sample_interfaces__msg__TwoEncoderStatus
    std::shared_ptr<sample_interfaces::msg::TwoEncoderStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TwoEncoderStatus_ & other) const
  {
    if (this->left != other.left) {
      return false;
    }
    if (this->right != other.right) {
      return false;
    }
    return true;
  }
  bool operator!=(const TwoEncoderStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TwoEncoderStatus_

// alias to use template instance with default allocator
using TwoEncoderStatus =
  sample_interfaces::msg::TwoEncoderStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace sample_interfaces

#endif  // SAMPLE_INTERFACES__MSG__DETAIL__TWO_ENCODER_STATUS__STRUCT_HPP_
