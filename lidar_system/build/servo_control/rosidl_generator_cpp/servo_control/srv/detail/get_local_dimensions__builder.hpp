// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from servo_control:srv/GetLocalDimensions.idl
// generated code does not contain a copyright notice

#ifndef SERVO_CONTROL__SRV__DETAIL__GET_LOCAL_DIMENSIONS__BUILDER_HPP_
#define SERVO_CONTROL__SRV__DETAIL__GET_LOCAL_DIMENSIONS__BUILDER_HPP_

#include "servo_control/srv/detail/get_local_dimensions__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace servo_control
{

namespace srv
{

namespace builder
{

class Init_GetLocalDimensions_Request_z
{
public:
  explicit Init_GetLocalDimensions_Request_z(::servo_control::srv::GetLocalDimensions_Request & msg)
  : msg_(msg)
  {}
  ::servo_control::srv::GetLocalDimensions_Request z(::servo_control::srv::GetLocalDimensions_Request::_z_type arg)
  {
    msg_.z = std::move(arg);
    return std::move(msg_);
  }

private:
  ::servo_control::srv::GetLocalDimensions_Request msg_;
};

class Init_GetLocalDimensions_Request_y
{
public:
  explicit Init_GetLocalDimensions_Request_y(::servo_control::srv::GetLocalDimensions_Request & msg)
  : msg_(msg)
  {}
  Init_GetLocalDimensions_Request_z y(::servo_control::srv::GetLocalDimensions_Request::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_GetLocalDimensions_Request_z(msg_);
  }

private:
  ::servo_control::srv::GetLocalDimensions_Request msg_;
};

class Init_GetLocalDimensions_Request_x
{
public:
  Init_GetLocalDimensions_Request_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetLocalDimensions_Request_y x(::servo_control::srv::GetLocalDimensions_Request::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_GetLocalDimensions_Request_y(msg_);
  }

private:
  ::servo_control::srv::GetLocalDimensions_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::servo_control::srv::GetLocalDimensions_Request>()
{
  return servo_control::srv::builder::Init_GetLocalDimensions_Request_x();
}

}  // namespace servo_control


namespace servo_control
{

namespace srv
{

namespace builder
{

class Init_GetLocalDimensions_Response_clearance_above
{
public:
  explicit Init_GetLocalDimensions_Response_clearance_above(::servo_control::srv::GetLocalDimensions_Response & msg)
  : msg_(msg)
  {}
  ::servo_control::srv::GetLocalDimensions_Response clearance_above(::servo_control::srv::GetLocalDimensions_Response::_clearance_above_type arg)
  {
    msg_.clearance_above = std::move(arg);
    return std::move(msg_);
  }

private:
  ::servo_control::srv::GetLocalDimensions_Response msg_;
};

class Init_GetLocalDimensions_Response_clearance_below
{
public:
  explicit Init_GetLocalDimensions_Response_clearance_below(::servo_control::srv::GetLocalDimensions_Response & msg)
  : msg_(msg)
  {}
  Init_GetLocalDimensions_Response_clearance_above clearance_below(::servo_control::srv::GetLocalDimensions_Response::_clearance_below_type arg)
  {
    msg_.clearance_below = std::move(arg);
    return Init_GetLocalDimensions_Response_clearance_above(msg_);
  }

private:
  ::servo_control::srv::GetLocalDimensions_Response msg_;
};

class Init_GetLocalDimensions_Response_room_height
{
public:
  explicit Init_GetLocalDimensions_Response_room_height(::servo_control::srv::GetLocalDimensions_Response & msg)
  : msg_(msg)
  {}
  Init_GetLocalDimensions_Response_clearance_below room_height(::servo_control::srv::GetLocalDimensions_Response::_room_height_type arg)
  {
    msg_.room_height = std::move(arg);
    return Init_GetLocalDimensions_Response_clearance_below(msg_);
  }

private:
  ::servo_control::srv::GetLocalDimensions_Response msg_;
};

class Init_GetLocalDimensions_Response_ceiling_z
{
public:
  explicit Init_GetLocalDimensions_Response_ceiling_z(::servo_control::srv::GetLocalDimensions_Response & msg)
  : msg_(msg)
  {}
  Init_GetLocalDimensions_Response_room_height ceiling_z(::servo_control::srv::GetLocalDimensions_Response::_ceiling_z_type arg)
  {
    msg_.ceiling_z = std::move(arg);
    return Init_GetLocalDimensions_Response_room_height(msg_);
  }

private:
  ::servo_control::srv::GetLocalDimensions_Response msg_;
};

class Init_GetLocalDimensions_Response_floor_z
{
public:
  explicit Init_GetLocalDimensions_Response_floor_z(::servo_control::srv::GetLocalDimensions_Response & msg)
  : msg_(msg)
  {}
  Init_GetLocalDimensions_Response_ceiling_z floor_z(::servo_control::srv::GetLocalDimensions_Response::_floor_z_type arg)
  {
    msg_.floor_z = std::move(arg);
    return Init_GetLocalDimensions_Response_ceiling_z(msg_);
  }

private:
  ::servo_control::srv::GetLocalDimensions_Response msg_;
};

class Init_GetLocalDimensions_Response_message
{
public:
  explicit Init_GetLocalDimensions_Response_message(::servo_control::srv::GetLocalDimensions_Response & msg)
  : msg_(msg)
  {}
  Init_GetLocalDimensions_Response_floor_z message(::servo_control::srv::GetLocalDimensions_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_GetLocalDimensions_Response_floor_z(msg_);
  }

private:
  ::servo_control::srv::GetLocalDimensions_Response msg_;
};

class Init_GetLocalDimensions_Response_success
{
public:
  Init_GetLocalDimensions_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetLocalDimensions_Response_message success(::servo_control::srv::GetLocalDimensions_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_GetLocalDimensions_Response_message(msg_);
  }

private:
  ::servo_control::srv::GetLocalDimensions_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::servo_control::srv::GetLocalDimensions_Response>()
{
  return servo_control::srv::builder::Init_GetLocalDimensions_Response_success();
}

}  // namespace servo_control

#endif  // SERVO_CONTROL__SRV__DETAIL__GET_LOCAL_DIMENSIONS__BUILDER_HPP_
