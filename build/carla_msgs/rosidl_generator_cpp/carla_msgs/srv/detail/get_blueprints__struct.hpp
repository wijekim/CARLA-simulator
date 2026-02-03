// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from carla_msgs:srv/GetBlueprints.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "carla_msgs/srv/get_blueprints.hpp"


#ifndef CARLA_MSGS__SRV__DETAIL__GET_BLUEPRINTS__STRUCT_HPP_
#define CARLA_MSGS__SRV__DETAIL__GET_BLUEPRINTS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__carla_msgs__srv__GetBlueprints_Request __attribute__((deprecated))
#else
# define DEPRECATED__carla_msgs__srv__GetBlueprints_Request __declspec(deprecated)
#endif

namespace carla_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetBlueprints_Request_
{
  using Type = GetBlueprints_Request_<ContainerAllocator>;

  explicit GetBlueprints_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->filter = "";
    }
  }

  explicit GetBlueprints_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : filter(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->filter = "";
    }
  }

  // field types and members
  using _filter_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _filter_type filter;

  // setters for named parameter idiom
  Type & set__filter(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->filter = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    carla_msgs::srv::GetBlueprints_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const carla_msgs::srv::GetBlueprints_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<carla_msgs::srv::GetBlueprints_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<carla_msgs::srv::GetBlueprints_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      carla_msgs::srv::GetBlueprints_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<carla_msgs::srv::GetBlueprints_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      carla_msgs::srv::GetBlueprints_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<carla_msgs::srv::GetBlueprints_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<carla_msgs::srv::GetBlueprints_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<carla_msgs::srv::GetBlueprints_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__carla_msgs__srv__GetBlueprints_Request
    std::shared_ptr<carla_msgs::srv::GetBlueprints_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__carla_msgs__srv__GetBlueprints_Request
    std::shared_ptr<carla_msgs::srv::GetBlueprints_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetBlueprints_Request_ & other) const
  {
    if (this->filter != other.filter) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetBlueprints_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetBlueprints_Request_

// alias to use template instance with default allocator
using GetBlueprints_Request =
  carla_msgs::srv::GetBlueprints_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace carla_msgs


#ifndef _WIN32
# define DEPRECATED__carla_msgs__srv__GetBlueprints_Response __attribute__((deprecated))
#else
# define DEPRECATED__carla_msgs__srv__GetBlueprints_Response __declspec(deprecated)
#endif

namespace carla_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetBlueprints_Response_
{
  using Type = GetBlueprints_Response_<ContainerAllocator>;

  explicit GetBlueprints_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit GetBlueprints_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _blueprints_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _blueprints_type blueprints;

  // setters for named parameter idiom
  Type & set__blueprints(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->blueprints = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    carla_msgs::srv::GetBlueprints_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const carla_msgs::srv::GetBlueprints_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<carla_msgs::srv::GetBlueprints_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<carla_msgs::srv::GetBlueprints_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      carla_msgs::srv::GetBlueprints_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<carla_msgs::srv::GetBlueprints_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      carla_msgs::srv::GetBlueprints_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<carla_msgs::srv::GetBlueprints_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<carla_msgs::srv::GetBlueprints_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<carla_msgs::srv::GetBlueprints_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__carla_msgs__srv__GetBlueprints_Response
    std::shared_ptr<carla_msgs::srv::GetBlueprints_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__carla_msgs__srv__GetBlueprints_Response
    std::shared_ptr<carla_msgs::srv::GetBlueprints_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetBlueprints_Response_ & other) const
  {
    if (this->blueprints != other.blueprints) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetBlueprints_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetBlueprints_Response_

// alias to use template instance with default allocator
using GetBlueprints_Response =
  carla_msgs::srv::GetBlueprints_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace carla_msgs


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__carla_msgs__srv__GetBlueprints_Event __attribute__((deprecated))
#else
# define DEPRECATED__carla_msgs__srv__GetBlueprints_Event __declspec(deprecated)
#endif

namespace carla_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetBlueprints_Event_
{
  using Type = GetBlueprints_Event_<ContainerAllocator>;

  explicit GetBlueprints_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit GetBlueprints_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<carla_msgs::srv::GetBlueprints_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<carla_msgs::srv::GetBlueprints_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<carla_msgs::srv::GetBlueprints_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<carla_msgs::srv::GetBlueprints_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<carla_msgs::srv::GetBlueprints_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<carla_msgs::srv::GetBlueprints_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<carla_msgs::srv::GetBlueprints_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<carla_msgs::srv::GetBlueprints_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    carla_msgs::srv::GetBlueprints_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const carla_msgs::srv::GetBlueprints_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<carla_msgs::srv::GetBlueprints_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<carla_msgs::srv::GetBlueprints_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      carla_msgs::srv::GetBlueprints_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<carla_msgs::srv::GetBlueprints_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      carla_msgs::srv::GetBlueprints_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<carla_msgs::srv::GetBlueprints_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<carla_msgs::srv::GetBlueprints_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<carla_msgs::srv::GetBlueprints_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__carla_msgs__srv__GetBlueprints_Event
    std::shared_ptr<carla_msgs::srv::GetBlueprints_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__carla_msgs__srv__GetBlueprints_Event
    std::shared_ptr<carla_msgs::srv::GetBlueprints_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetBlueprints_Event_ & other) const
  {
    if (this->info != other.info) {
      return false;
    }
    if (this->request != other.request) {
      return false;
    }
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetBlueprints_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetBlueprints_Event_

// alias to use template instance with default allocator
using GetBlueprints_Event =
  carla_msgs::srv::GetBlueprints_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace carla_msgs

namespace carla_msgs
{

namespace srv
{

struct GetBlueprints
{
  using Request = carla_msgs::srv::GetBlueprints_Request;
  using Response = carla_msgs::srv::GetBlueprints_Response;
  using Event = carla_msgs::srv::GetBlueprints_Event;
};

}  // namespace srv

}  // namespace carla_msgs

#endif  // CARLA_MSGS__SRV__DETAIL__GET_BLUEPRINTS__STRUCT_HPP_
