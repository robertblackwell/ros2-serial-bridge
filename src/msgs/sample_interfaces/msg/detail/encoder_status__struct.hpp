// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sample_interfaces:msg/EncoderStatus.idl
// generated code does not contain a copyright notice

#ifndef SAMPLE_INTERFACES__MSG__DETAIL__ENCODER_STATUS__STRUCT_HPP_
#define SAMPLE_INTERFACES__MSG__DETAIL__ENCODER_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__sample_interfaces__msg__EncoderStatus __attribute__((deprecated))
#else
# define DEPRECATED__sample_interfaces__msg__EncoderStatus __declspec(deprecated)
#endif

namespace sample_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct EncoderStatus_
{
  using Type = EncoderStatus_<ContainerAllocator>;

  explicit EncoderStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sample_sum = 0ll;
      this->sample_time_stamp_usecs = 0ll;
      this->motor_rpm_estimate = 0.0f;
      this->direction_pin_state = 0;
    }
  }

  explicit EncoderStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sample_sum = 0ll;
      this->sample_time_stamp_usecs = 0ll;
      this->motor_rpm_estimate = 0.0f;
      this->direction_pin_state = 0;
    }
  }

  // field types and members
  using _sample_sum_type =
    int64_t;
  _sample_sum_type sample_sum;
  using _sample_time_stamp_usecs_type =
    int64_t;
  _sample_time_stamp_usecs_type sample_time_stamp_usecs;
  using _motor_rpm_estimate_type =
    float;
  _motor_rpm_estimate_type motor_rpm_estimate;
  using _direction_pin_state_type =
    uint8_t;
  _direction_pin_state_type direction_pin_state;

  // setters for named parameter idiom
  Type & set__sample_sum(
    const int64_t & _arg)
  {
    this->sample_sum = _arg;
    return *this;
  }
  Type & set__sample_time_stamp_usecs(
    const int64_t & _arg)
  {
    this->sample_time_stamp_usecs = _arg;
    return *this;
  }
  Type & set__motor_rpm_estimate(
    const float & _arg)
  {
    this->motor_rpm_estimate = _arg;
    return *this;
  }
  Type & set__direction_pin_state(
    const uint8_t & _arg)
  {
    this->direction_pin_state = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    sample_interfaces::msg::EncoderStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const sample_interfaces::msg::EncoderStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sample_interfaces::msg::EncoderStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sample_interfaces::msg::EncoderStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sample_interfaces::msg::EncoderStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sample_interfaces::msg::EncoderStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sample_interfaces::msg::EncoderStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sample_interfaces::msg::EncoderStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sample_interfaces::msg::EncoderStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sample_interfaces::msg::EncoderStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sample_interfaces__msg__EncoderStatus
    std::shared_ptr<sample_interfaces::msg::EncoderStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sample_interfaces__msg__EncoderStatus
    std::shared_ptr<sample_interfaces::msg::EncoderStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const EncoderStatus_ & other) const
  {
    if (this->sample_sum != other.sample_sum) {
      return false;
    }
    if (this->sample_time_stamp_usecs != other.sample_time_stamp_usecs) {
      return false;
    }
    if (this->motor_rpm_estimate != other.motor_rpm_estimate) {
      return false;
    }
    if (this->direction_pin_state != other.direction_pin_state) {
      return false;
    }
    return true;
  }
  bool operator!=(const EncoderStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct EncoderStatus_

// alias to use template instance with default allocator
using EncoderStatus =
  sample_interfaces::msg::EncoderStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace sample_interfaces

#endif  // SAMPLE_INTERFACES__MSG__DETAIL__ENCODER_STATUS__STRUCT_HPP_
