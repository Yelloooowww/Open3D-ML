// Generated by gencpp from file subt_msgs/node_statRequest.msg
// DO NOT EDIT!


#ifndef SUBT_MSGS_MESSAGE_NODE_STATREQUEST_H
#define SUBT_MSGS_MESSAGE_NODE_STATREQUEST_H


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
struct node_statRequest_
{
  typedef node_statRequest_<ContainerAllocator> Type;

  node_statRequest_()
    : count(0)  {
    }
  node_statRequest_(const ContainerAllocator& _alloc)
    : count(0)  {
  (void)_alloc;
    }



   typedef int32_t _count_type;
  _count_type count;





  typedef boost::shared_ptr< ::subt_msgs::node_statRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::subt_msgs::node_statRequest_<ContainerAllocator> const> ConstPtr;

}; // struct node_statRequest_

typedef ::subt_msgs::node_statRequest_<std::allocator<void> > node_statRequest;

typedef boost::shared_ptr< ::subt_msgs::node_statRequest > node_statRequestPtr;
typedef boost::shared_ptr< ::subt_msgs::node_statRequest const> node_statRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::subt_msgs::node_statRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::subt_msgs::node_statRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::subt_msgs::node_statRequest_<ContainerAllocator1> & lhs, const ::subt_msgs::node_statRequest_<ContainerAllocator2> & rhs)
{
  return lhs.count == rhs.count;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::subt_msgs::node_statRequest_<ContainerAllocator1> & lhs, const ::subt_msgs::node_statRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace subt_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::subt_msgs::node_statRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::subt_msgs::node_statRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::subt_msgs::node_statRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::subt_msgs::node_statRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::subt_msgs::node_statRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::subt_msgs::node_statRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::subt_msgs::node_statRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "602d642babe509c7c59f497c23e716a9";
  }

  static const char* value(const ::subt_msgs::node_statRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x602d642babe509c7ULL;
  static const uint64_t static_value2 = 0xc59f497c23e716a9ULL;
};

template<class ContainerAllocator>
struct DataType< ::subt_msgs::node_statRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "subt_msgs/node_statRequest";
  }

  static const char* value(const ::subt_msgs::node_statRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::subt_msgs::node_statRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "#Request data\n"
"int32 count\n"
;
  }

  static const char* value(const ::subt_msgs::node_statRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::subt_msgs::node_statRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.count);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct node_statRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::subt_msgs::node_statRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::subt_msgs::node_statRequest_<ContainerAllocator>& v)
  {
    s << indent << "count: ";
    Printer<int32_t>::stream(s, indent + "  ", v.count);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SUBT_MSGS_MESSAGE_NODE_STATREQUEST_H
