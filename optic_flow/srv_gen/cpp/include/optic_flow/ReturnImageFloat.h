/* Auto-generated by genmsg_cpp for file /home/caveman/ros_workspace/flybot/optic_flow/srv/ReturnImageFloat.srv */
#ifndef OPTIC_FLOW_SERVICE_RETURNIMAGEFLOAT_H
#define OPTIC_FLOW_SERVICE_RETURNIMAGEFLOAT_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "ros/service_traits.h"




namespace optic_flow
{
template <class ContainerAllocator>
struct ReturnImageFloatRequest_ {
  typedef ReturnImageFloatRequest_<ContainerAllocator> Type;

  ReturnImageFloatRequest_()
  : request(false)
  {
  }

  ReturnImageFloatRequest_(const ContainerAllocator& _alloc)
  : request(false)
  {
  }

  typedef uint8_t _request_type;
  uint8_t request;


  typedef boost::shared_ptr< ::optic_flow::ReturnImageFloatRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::optic_flow::ReturnImageFloatRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct ReturnImageFloatRequest
typedef  ::optic_flow::ReturnImageFloatRequest_<std::allocator<void> > ReturnImageFloatRequest;

typedef boost::shared_ptr< ::optic_flow::ReturnImageFloatRequest> ReturnImageFloatRequestPtr;
typedef boost::shared_ptr< ::optic_flow::ReturnImageFloatRequest const> ReturnImageFloatRequestConstPtr;


template <class ContainerAllocator>
struct ReturnImageFloatResponse_ {
  typedef ReturnImageFloatResponse_<ContainerAllocator> Type;

  ReturnImageFloatResponse_()
  : shape()
  , data()
  {
  }

  ReturnImageFloatResponse_(const ContainerAllocator& _alloc)
  : shape(_alloc)
  , data(_alloc)
  {
  }

  typedef std::vector<uint16_t, typename ContainerAllocator::template rebind<uint16_t>::other >  _shape_type;
  std::vector<uint16_t, typename ContainerAllocator::template rebind<uint16_t>::other >  shape;

  typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _data_type;
  std::vector<float, typename ContainerAllocator::template rebind<float>::other >  data;


  typedef boost::shared_ptr< ::optic_flow::ReturnImageFloatResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::optic_flow::ReturnImageFloatResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct ReturnImageFloatResponse
typedef  ::optic_flow::ReturnImageFloatResponse_<std::allocator<void> > ReturnImageFloatResponse;

typedef boost::shared_ptr< ::optic_flow::ReturnImageFloatResponse> ReturnImageFloatResponsePtr;
typedef boost::shared_ptr< ::optic_flow::ReturnImageFloatResponse const> ReturnImageFloatResponseConstPtr;

struct ReturnImageFloat
{

typedef ReturnImageFloatRequest Request;
typedef ReturnImageFloatResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct ReturnImageFloat
} // namespace optic_flow

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::optic_flow::ReturnImageFloatRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::optic_flow::ReturnImageFloatRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::optic_flow::ReturnImageFloatRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "6f7e5ad6ab0ddf42c5727a195315a470";
  }

  static const char* value(const  ::optic_flow::ReturnImageFloatRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x6f7e5ad6ab0ddf42ULL;
  static const uint64_t static_value2 = 0xc5727a195315a470ULL;
};

template<class ContainerAllocator>
struct DataType< ::optic_flow::ReturnImageFloatRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "optic_flow/ReturnImageFloatRequest";
  }

  static const char* value(const  ::optic_flow::ReturnImageFloatRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::optic_flow::ReturnImageFloatRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "bool request\n\
\n\
";
  }

  static const char* value(const  ::optic_flow::ReturnImageFloatRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::optic_flow::ReturnImageFloatRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::optic_flow::ReturnImageFloatResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::optic_flow::ReturnImageFloatResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::optic_flow::ReturnImageFloatResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "b44a887daa94a23c9a22247bea6d0b5d";
  }

  static const char* value(const  ::optic_flow::ReturnImageFloatResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xb44a887daa94a23cULL;
  static const uint64_t static_value2 = 0x9a22247bea6d0b5dULL;
};

template<class ContainerAllocator>
struct DataType< ::optic_flow::ReturnImageFloatResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "optic_flow/ReturnImageFloatResponse";
  }

  static const char* value(const  ::optic_flow::ReturnImageFloatResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::optic_flow::ReturnImageFloatResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "uint16[] shape\n\
float32[] data\n\
\n\
\n\
";
  }

  static const char* value(const  ::optic_flow::ReturnImageFloatResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::optic_flow::ReturnImageFloatRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.request);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct ReturnImageFloatRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::optic_flow::ReturnImageFloatResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.shape);
    stream.next(m.data);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct ReturnImageFloatResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<optic_flow::ReturnImageFloat> {
  static const char* value() 
  {
    return "6cd7e2eb7fd7a9c18f076cc4104651f8";
  }

  static const char* value(const optic_flow::ReturnImageFloat&) { return value(); } 
};

template<>
struct DataType<optic_flow::ReturnImageFloat> {
  static const char* value() 
  {
    return "optic_flow/ReturnImageFloat";
  }

  static const char* value(const optic_flow::ReturnImageFloat&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<optic_flow::ReturnImageFloatRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "6cd7e2eb7fd7a9c18f076cc4104651f8";
  }

  static const char* value(const optic_flow::ReturnImageFloatRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<optic_flow::ReturnImageFloatRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "optic_flow/ReturnImageFloat";
  }

  static const char* value(const optic_flow::ReturnImageFloatRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<optic_flow::ReturnImageFloatResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "6cd7e2eb7fd7a9c18f076cc4104651f8";
  }

  static const char* value(const optic_flow::ReturnImageFloatResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<optic_flow::ReturnImageFloatResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "optic_flow/ReturnImageFloat";
  }

  static const char* value(const optic_flow::ReturnImageFloatResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // OPTIC_FLOW_SERVICE_RETURNIMAGEFLOAT_H
