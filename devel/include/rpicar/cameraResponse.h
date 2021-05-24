// Generated by gencpp from file rpicar/cameraResponse.msg
// DO NOT EDIT!


#ifndef RPICAR_MESSAGE_CAMERARESPONSE_H
#define RPICAR_MESSAGE_CAMERARESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace rpicar
{
template <class ContainerAllocator>
struct cameraResponse_
{
  typedef cameraResponse_<ContainerAllocator> Type;

  cameraResponse_()
    : success(false)
    , status_message()  {
    }
  cameraResponse_(const ContainerAllocator& _alloc)
    : success(false)
    , status_message(_alloc)  {
  (void)_alloc;
    }



   typedef uint8_t _success_type;
  _success_type success;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _status_message_type;
  _status_message_type status_message;





  typedef boost::shared_ptr< ::rpicar::cameraResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rpicar::cameraResponse_<ContainerAllocator> const> ConstPtr;

}; // struct cameraResponse_

typedef ::rpicar::cameraResponse_<std::allocator<void> > cameraResponse;

typedef boost::shared_ptr< ::rpicar::cameraResponse > cameraResponsePtr;
typedef boost::shared_ptr< ::rpicar::cameraResponse const> cameraResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::rpicar::cameraResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::rpicar::cameraResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::rpicar::cameraResponse_<ContainerAllocator1> & lhs, const ::rpicar::cameraResponse_<ContainerAllocator2> & rhs)
{
  return lhs.success == rhs.success &&
    lhs.status_message == rhs.status_message;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::rpicar::cameraResponse_<ContainerAllocator1> & lhs, const ::rpicar::cameraResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace rpicar

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::rpicar::cameraResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rpicar::cameraResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rpicar::cameraResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rpicar::cameraResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rpicar::cameraResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rpicar::cameraResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::rpicar::cameraResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "2ec6f3eff0161f4257b808b12bc830c2";
  }

  static const char* value(const ::rpicar::cameraResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x2ec6f3eff0161f42ULL;
  static const uint64_t static_value2 = 0x57b808b12bc830c2ULL;
};

template<class ContainerAllocator>
struct DataType< ::rpicar::cameraResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "rpicar/cameraResponse";
  }

  static const char* value(const ::rpicar::cameraResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::rpicar::cameraResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool success\n"
"string status_message \n"
;
  }

  static const char* value(const ::rpicar::cameraResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::rpicar::cameraResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.success);
      stream.next(m.status_message);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct cameraResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::rpicar::cameraResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::rpicar::cameraResponse_<ContainerAllocator>& v)
  {
    s << indent << "success: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.success);
    s << indent << "status_message: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.status_message);
  }
};

} // namespace message_operations
} // namespace ros

#endif // RPICAR_MESSAGE_CAMERARESPONSE_H
