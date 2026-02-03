// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from carla_msgs:srv/GetBlueprints.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "carla_msgs/srv/get_blueprints.hpp"


#ifndef CARLA_MSGS__SRV__DETAIL__GET_BLUEPRINTS__BUILDER_HPP_
#define CARLA_MSGS__SRV__DETAIL__GET_BLUEPRINTS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "carla_msgs/srv/detail/get_blueprints__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace carla_msgs
{

namespace srv
{

namespace builder
{

class Init_GetBlueprints_Request_filter
{
public:
  Init_GetBlueprints_Request_filter()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::carla_msgs::srv::GetBlueprints_Request filter(::carla_msgs::srv::GetBlueprints_Request::_filter_type arg)
  {
    msg_.filter = std::move(arg);
    return std::move(msg_);
  }

private:
  ::carla_msgs::srv::GetBlueprints_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::carla_msgs::srv::GetBlueprints_Request>()
{
  return carla_msgs::srv::builder::Init_GetBlueprints_Request_filter();
}

}  // namespace carla_msgs


namespace carla_msgs
{

namespace srv
{

namespace builder
{

class Init_GetBlueprints_Response_blueprints
{
public:
  Init_GetBlueprints_Response_blueprints()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::carla_msgs::srv::GetBlueprints_Response blueprints(::carla_msgs::srv::GetBlueprints_Response::_blueprints_type arg)
  {
    msg_.blueprints = std::move(arg);
    return std::move(msg_);
  }

private:
  ::carla_msgs::srv::GetBlueprints_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::carla_msgs::srv::GetBlueprints_Response>()
{
  return carla_msgs::srv::builder::Init_GetBlueprints_Response_blueprints();
}

}  // namespace carla_msgs


namespace carla_msgs
{

namespace srv
{

namespace builder
{

class Init_GetBlueprints_Event_response
{
public:
  explicit Init_GetBlueprints_Event_response(::carla_msgs::srv::GetBlueprints_Event & msg)
  : msg_(msg)
  {}
  ::carla_msgs::srv::GetBlueprints_Event response(::carla_msgs::srv::GetBlueprints_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::carla_msgs::srv::GetBlueprints_Event msg_;
};

class Init_GetBlueprints_Event_request
{
public:
  explicit Init_GetBlueprints_Event_request(::carla_msgs::srv::GetBlueprints_Event & msg)
  : msg_(msg)
  {}
  Init_GetBlueprints_Event_response request(::carla_msgs::srv::GetBlueprints_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_GetBlueprints_Event_response(msg_);
  }

private:
  ::carla_msgs::srv::GetBlueprints_Event msg_;
};

class Init_GetBlueprints_Event_info
{
public:
  Init_GetBlueprints_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetBlueprints_Event_request info(::carla_msgs::srv::GetBlueprints_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_GetBlueprints_Event_request(msg_);
  }

private:
  ::carla_msgs::srv::GetBlueprints_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::carla_msgs::srv::GetBlueprints_Event>()
{
  return carla_msgs::srv::builder::Init_GetBlueprints_Event_info();
}

}  // namespace carla_msgs

#endif  // CARLA_MSGS__SRV__DETAIL__GET_BLUEPRINTS__BUILDER_HPP_
