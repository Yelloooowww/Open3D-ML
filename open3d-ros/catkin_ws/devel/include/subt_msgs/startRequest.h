// Generated by gencpp from file subt_msgs/startRequest.msg
// DO NOT EDIT!


#ifndef SUBT_MSGS_MESSAGE_STARTREQUEST_H
#define SUBT_MSGS_MESSAGE_STARTREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace subt_msgs
{
template <class ContainerAllocator>
struct startRequest_
{
  typedef startRequest_<ContainerAllocator> Type;

  startRequest_()
    {
    }
  startRequest_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::subt_msgs::startRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::subt_msgs::startRequest_<ContainerAllocator> const> ConstPtr;

}; // struct startRequest_

typedef ::subt_msgs::startRequest_<std::allocator<void> > startRequest;

typedef boost::shared_ptr< ::subt_msgs::startRequest > startRequestPtr;
typedef boost::shared_ptr< ::subt_msgs::startRequest const> startRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::subt_msgs::startRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::subt_msgs::startRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


} // namespace subt_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::subt_msgs::startRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::subt_msgs::startRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::subt_msgs::startRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::subt_msgs::startRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::subt_msgs::startRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::subt_msgs::startRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::subt_msgs::startRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::subt_msgs::startRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::subt_msgs::startRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "subt_msgs/startRequest";
  }

  static const char* value(const ::subt_msgs::startRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::subt_msgs::startRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "#Request data\n"
;
  }

  static const char* value(const ::subt_msgs::startRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::subt_msgs::startRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct startRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::subt_msgs::startRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::subt_msgs::startRequest_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // SUBT_MSGS_MESSAGE_STARTREQUEST_H
