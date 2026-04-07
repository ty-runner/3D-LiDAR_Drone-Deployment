// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from servo_control:srv/GetLocalDimensions.idl
// generated code does not contain a copyright notice

#ifndef SERVO_CONTROL__SRV__DETAIL__GET_LOCAL_DIMENSIONS__TRAITS_HPP_
#define SERVO_CONTROL__SRV__DETAIL__GET_LOCAL_DIMENSIONS__TRAITS_HPP_

#include "servo_control/srv/detail/get_local_dimensions__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<servo_control::srv::GetLocalDimensions_Request>()
{
  return "servo_control::srv::GetLocalDimensions_Request";
}

template<>
inline const char * name<servo_control::srv::GetLocalDimensions_Request>()
{
  return "servo_control/srv/GetLocalDimensions_Request";
}

template<>
struct has_fixed_size<servo_control::srv::GetLocalDimensions_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<servo_control::srv::GetLocalDimensions_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<servo_control::srv::GetLocalDimensions_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<servo_control::srv::GetLocalDimensions_Response>()
{
  return "servo_control::srv::GetLocalDimensions_Response";
}

template<>
inline const char * name<servo_control::srv::GetLocalDimensions_Response>()
{
  return "servo_control/srv/GetLocalDimensions_Response";
}

template<>
struct has_fixed_size<servo_control::srv::GetLocalDimensions_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<servo_control::srv::GetLocalDimensions_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<servo_control::srv::GetLocalDimensions_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<servo_control::srv::GetLocalDimensions>()
{
  return "servo_control::srv::GetLocalDimensions";
}

template<>
inline const char * name<servo_control::srv::GetLocalDimensions>()
{
  return "servo_control/srv/GetLocalDimensions";
}

template<>
struct has_fixed_size<servo_control::srv::GetLocalDimensions>
  : std::integral_constant<
    bool,
    has_fixed_size<servo_control::srv::GetLocalDimensions_Request>::value &&
    has_fixed_size<servo_control::srv::GetLocalDimensions_Response>::value
  >
{
};

template<>
struct has_bounded_size<servo_control::srv::GetLocalDimensions>
  : std::integral_constant<
    bool,
    has_bounded_size<servo_control::srv::GetLocalDimensions_Request>::value &&
    has_bounded_size<servo_control::srv::GetLocalDimensions_Response>::value
  >
{
};

template<>
struct is_service<servo_control::srv::GetLocalDimensions>
  : std::true_type
{
};

template<>
struct is_service_request<servo_control::srv::GetLocalDimensions_Request>
  : std::true_type
{
};

template<>
struct is_service_response<servo_control::srv::GetLocalDimensions_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // SERVO_CONTROL__SRV__DETAIL__GET_LOCAL_DIMENSIONS__TRAITS_HPP_
