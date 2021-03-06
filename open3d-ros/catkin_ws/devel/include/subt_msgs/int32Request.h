// Generated by gencpp from file subt_msgs/int32Request.msg
// DO NOT EDIT!


#ifndef SUBT_MSGS_MESSAGE_INT32REQUEST_H
#define SUBT_MSGS_MESSAGE_INT32REQUEST_H


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
struct int32Request_
{
  typedef int32Request_<ContainerAllocator> Type;

  int32Request_()
    : data(0)  {
    }
  int32Request_(const ContainerAllocator& _alloc)
    : data(0)  {
  (void)_alloc;
    }



   typedef int32_t _data_type;
  _data_type data;





  typedef boost::shared_ptr< ::subt_msgs::int32Request_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::subt_msgs::int32Request_<ContainerAllocator> const> ConstPtr;

}; // struct int32Request_

typedef ::subt_msgs::int32Request_<std::allocator<void> > int32Request;

typedef boost::shared_ptr< ::subt_msgs::int32Request > int32RequestPtr;
typedef boost::shared_ptr< ::subt_msgs::int32Request const> int32RequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::subt_msgs::int32Request_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::subt_msgs::int32Request_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::subt_msgs::int32Request_<ContainerAllocator1> & lhs, const ::subt_msgs::int32Request_<ContainerAllocator2> & rhs)
{
  return lhs.data == rhs.data;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::subt_msgs::int32Request_<ContainerAllocator1> & lhs, const ::subt_msgs::int32Request_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace subt_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::subt_msgs::int32Request_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::subt_msgs::int32Request_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::subt_msgs::int32Request_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::subt_msgs::int32Request_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::subt_msgs::int32Request_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::subt_msgs::int32Request_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::subt_msgs::int32Request_<ContainerAllocator> >
{
  static const char* value()
  {
    return "da5909fbe378aeaf85e547e830cc1bb7";
  }

  static const char* value(const ::subt_msgs::int32Request_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xda5909fbe378aeafULL;
  static const uint64_t static_value2 = 0x85e547e830cc1bb7ULL;
};

template<class ContainerAllocator>
struct DataType< ::subt_msgs::int32Request_<ContainerAllocator> >
{
  static const char* value()
  {
    return "subt_msgs/int32Request";
  }

  static const char* value(const ::subt_msgs::int32Request_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::subt_msgs::int32Request_<ContainerAllocator> >
{
  static const char* value()
  {
    return "#Request data\n"
"int32 data\n"
;
  }

  static const char* value(const ::subt_msgs::int32Request_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::subt_msgs::int32Request_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.data);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct int32Request_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::subt_msgs::int32Request_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::subt_msgs::int32Request_<ContainerAllocator>& v)
  {
    s << indent << "data: ";
    Printer<int32_t>::stream(s, indent + "  ", v.data);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SUBT_MSGS_MESSAGE_INT32REQUEST_H
