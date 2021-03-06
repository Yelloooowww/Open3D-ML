// Generated by gencpp from file subt_msgs/report.msg
// DO NOT EDIT!


#ifndef SUBT_MSGS_MESSAGE_REPORT_H
#define SUBT_MSGS_MESSAGE_REPORT_H

#include <ros/service_traits.h>


#include <subt_msgs/reportRequest.h>
#include <subt_msgs/reportResponse.h>


namespace subt_msgs
{

struct report
{

typedef reportRequest Request;
typedef reportResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct report
} // namespace subt_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::subt_msgs::report > {
  static const char* value()
  {
    return "d571fa3f7149c57374206fa01faa05ff";
  }

  static const char* value(const ::subt_msgs::report&) { return value(); }
};

template<>
struct DataType< ::subt_msgs::report > {
  static const char* value()
  {
    return "subt_msgs/report";
  }

  static const char* value(const ::subt_msgs::report&) { return value(); }
};


// service_traits::MD5Sum< ::subt_msgs::reportRequest> should match
// service_traits::MD5Sum< ::subt_msgs::report >
template<>
struct MD5Sum< ::subt_msgs::reportRequest>
{
  static const char* value()
  {
    return MD5Sum< ::subt_msgs::report >::value();
  }
  static const char* value(const ::subt_msgs::reportRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::subt_msgs::reportRequest> should match
// service_traits::DataType< ::subt_msgs::report >
template<>
struct DataType< ::subt_msgs::reportRequest>
{
  static const char* value()
  {
    return DataType< ::subt_msgs::report >::value();
  }
  static const char* value(const ::subt_msgs::reportRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::subt_msgs::reportResponse> should match
// service_traits::MD5Sum< ::subt_msgs::report >
template<>
struct MD5Sum< ::subt_msgs::reportResponse>
{
  static const char* value()
  {
    return MD5Sum< ::subt_msgs::report >::value();
  }
  static const char* value(const ::subt_msgs::reportResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::subt_msgs::reportResponse> should match
// service_traits::DataType< ::subt_msgs::report >
template<>
struct DataType< ::subt_msgs::reportResponse>
{
  static const char* value()
  {
    return DataType< ::subt_msgs::report >::value();
  }
  static const char* value(const ::subt_msgs::reportResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // SUBT_MSGS_MESSAGE_REPORT_H
