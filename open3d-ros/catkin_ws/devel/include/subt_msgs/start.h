// Generated by gencpp from file subt_msgs/start.msg
// DO NOT EDIT!


#ifndef SUBT_MSGS_MESSAGE_START_H
#define SUBT_MSGS_MESSAGE_START_H

#include <ros/service_traits.h>


#include <subt_msgs/startRequest.h>
#include <subt_msgs/startResponse.h>


namespace subt_msgs
{

struct start
{

typedef startRequest Request;
typedef startResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct start
} // namespace subt_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::subt_msgs::start > {
  static const char* value()
  {
    return "c22f2a1ed8654a0b365f1bb3f7ff2c0f";
  }

  static const char* value(const ::subt_msgs::start&) { return value(); }
};

template<>
struct DataType< ::subt_msgs::start > {
  static const char* value()
  {
    return "subt_msgs/start";
  }

  static const char* value(const ::subt_msgs::start&) { return value(); }
};


// service_traits::MD5Sum< ::subt_msgs::startRequest> should match
// service_traits::MD5Sum< ::subt_msgs::start >
template<>
struct MD5Sum< ::subt_msgs::startRequest>
{
  static const char* value()
  {
    return MD5Sum< ::subt_msgs::start >::value();
  }
  static const char* value(const ::subt_msgs::startRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::subt_msgs::startRequest> should match
// service_traits::DataType< ::subt_msgs::start >
template<>
struct DataType< ::subt_msgs::startRequest>
{
  static const char* value()
  {
    return DataType< ::subt_msgs::start >::value();
  }
  static const char* value(const ::subt_msgs::startRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::subt_msgs::startResponse> should match
// service_traits::MD5Sum< ::subt_msgs::start >
template<>
struct MD5Sum< ::subt_msgs::startResponse>
{
  static const char* value()
  {
    return MD5Sum< ::subt_msgs::start >::value();
  }
  static const char* value(const ::subt_msgs::startResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::subt_msgs::startResponse> should match
// service_traits::DataType< ::subt_msgs::start >
template<>
struct DataType< ::subt_msgs::startResponse>
{
  static const char* value()
  {
    return DataType< ::subt_msgs::start >::value();
  }
  static const char* value(const ::subt_msgs::startResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // SUBT_MSGS_MESSAGE_START_H
