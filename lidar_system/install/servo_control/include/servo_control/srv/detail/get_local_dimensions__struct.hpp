// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from servo_control:srv/GetLocalDimensions.idl
// generated code does not contain a copyright notice

#ifndef SERVO_CONTROL__SRV__DETAIL__GET_LOCAL_DIMENSIONS__STRUCT_HPP_
#define SERVO_CONTROL__SRV__DETAIL__GET_LOCAL_DIMENSIONS__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__servo_control__srv__GetLocalDimensions_Request __attribute__((deprecated))
#else
# define DEPRECATED__servo_control__srv__GetLocalDimensions_Request __declspec(deprecated)
#endif

namespace servo_control
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetLocalDimensions_Request_
{
  using Type = GetLocalDimensions_Request_<ContainerAllocator>;

  explicit GetLocalDimensions_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0;
      this->y = 0.0;
      this->z = 0.0;
    }
  }

  explicit GetLocalDimensions_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0;
      this->y = 0.0;
      this->z = 0.0;
    }
  }

  // field types and members
  using _x_type =
    double;
  _x_type x;
  using _y_type =
    double;
  _y_type y;
  using _z_type =
    double;
  _z_type z;

  // setters for named parameter idiom
  Type & set__x(
    const double & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const double & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__z(
    const double & _arg)
  {
    this->z = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    servo_control::srv::GetLocalDimensions_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const servo_control::srv::GetLocalDimensions_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<servo_control::srv::GetLocalDimensions_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<servo_control::srv::GetLocalDimensions_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      servo_control::srv::GetLocalDimensions_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<servo_control::srv::GetLocalDimensions_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      servo_control::srv::GetLocalDimensions_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<servo_control::srv::GetLocalDimensions_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<servo_control::srv::GetLocalDimensions_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<servo_control::srv::GetLocalDimensions_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__servo_control__srv__GetLocalDimensions_Request
    std::shared_ptr<servo_control::srv::GetLocalDimensions_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__servo_control__srv__GetLocalDimensions_Request
    std::shared_ptr<servo_control::srv::GetLocalDimensions_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetLocalDimensions_Request_ & other) const
  {
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->z != other.z) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetLocalDimensions_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetLocalDimensions_Request_

// alias to use template instance with default allocator
using GetLocalDimensions_Request =
  servo_control::srv::GetLocalDimensions_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace servo_control


#ifndef _WIN32
# define DEPRECATED__servo_control__srv__GetLocalDimensions_Response __attribute__((deprecated))
#else
# define DEPRECATED__servo_control__srv__GetLocalDimensions_Response __declspec(deprecated)
#endif

namespace servo_control
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetLocalDimensions_Response_
{
  using Type = GetLocalDimensions_Response_<ContainerAllocator>;

  explicit GetLocalDimensions_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
      this->floor_z = 0.0;
      this->ceiling_z = 0.0;
      this->room_height = 0.0;
      this->clearance_below = 0.0;
      this->clearance_above = 0.0;
    }
  }

  explicit GetLocalDimensions_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
      this->floor_z = 0.0;
      this->ceiling_z = 0.0;
      this->room_height = 0.0;
      this->clearance_below = 0.0;
      this->clearance_above = 0.0;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _message_type message;
  using _floor_z_type =
    double;
  _floor_z_type floor_z;
  using _ceiling_z_type =
    double;
  _ceiling_z_type ceiling_z;
  using _room_height_type =
    double;
  _room_height_type room_height;
  using _clearance_below_type =
    double;
  _clearance_below_type clearance_below;
  using _clearance_above_type =
    double;
  _clearance_above_type clearance_above;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->message = _arg;
    return *this;
  }
  Type & set__floor_z(
    const double & _arg)
  {
    this->floor_z = _arg;
    return *this;
  }
  Type & set__ceiling_z(
    const double & _arg)
  {
    this->ceiling_z = _arg;
    return *this;
  }
  Type & set__room_height(
    const double & _arg)
  {
    this->room_height = _arg;
    return *this;
  }
  Type & set__clearance_below(
    const double & _arg)
  {
    this->clearance_below = _arg;
    return *this;
  }
  Type & set__clearance_above(
    const double & _arg)
  {
    this->clearance_above = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    servo_control::srv::GetLocalDimensions_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const servo_control::srv::GetLocalDimensions_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<servo_control::srv::GetLocalDimensions_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<servo_control::srv::GetLocalDimensions_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      servo_control::srv::GetLocalDimensions_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<servo_control::srv::GetLocalDimensions_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      servo_control::srv::GetLocalDimensions_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<servo_control::srv::GetLocalDimensions_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<servo_control::srv::GetLocalDimensions_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<servo_control::srv::GetLocalDimensions_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__servo_control__srv__GetLocalDimensions_Response
    std::shared_ptr<servo_control::srv::GetLocalDimensions_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__servo_control__srv__GetLocalDimensions_Response
    std::shared_ptr<servo_control::srv::GetLocalDimensions_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetLocalDimensions_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    if (this->floor_z != other.floor_z) {
      return false;
    }
    if (this->ceiling_z != other.ceiling_z) {
      return false;
    }
    if (this->room_height != other.room_height) {
      return false;
    }
    if (this->clearance_below != other.clearance_below) {
      return false;
    }
    if (this->clearance_above != other.clearance_above) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetLocalDimensions_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetLocalDimensions_Response_

// alias to use template instance with default allocator
using GetLocalDimensions_Response =
  servo_control::srv::GetLocalDimensions_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace servo_control

namespace servo_control
{

namespace srv
{

struct GetLocalDimensions
{
  using Request = servo_control::srv::GetLocalDimensions_Request;
  using Response = servo_control::srv::GetLocalDimensions_Response;
};

}  // namespace srv

}  // namespace servo_control

#endif  // SERVO_CONTROL__SRV__DETAIL__GET_LOCAL_DIMENSIONS__STRUCT_HPP_
