// Generated by gencpp from file gator_msgs/DriveState.msg
// DO NOT EDIT!


#ifndef GATOR_MSGS_MESSAGE_DRIVESTATE_H
#define GATOR_MSGS_MESSAGE_DRIVESTATE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace gator_msgs
{
template <class ContainerAllocator>
struct DriveState_
{
  typedef DriveState_<ContainerAllocator> Type;

  DriveState_()
    : steeringangle(0.0)
    , wheelvelocity(0.0)
    , brake(false)  {
    }
  DriveState_(const ContainerAllocator& _alloc)
    : steeringangle(0.0)
    , wheelvelocity(0.0)
    , brake(false)  {
    }



   typedef float _steeringangle_type;
  _steeringangle_type steeringangle;

   typedef float _wheelvelocity_type;
  _wheelvelocity_type wheelvelocity;

   typedef uint8_t _brake_type;
  _brake_type brake;




  typedef boost::shared_ptr< ::gator_msgs::DriveState_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::gator_msgs::DriveState_<ContainerAllocator> const> ConstPtr;

}; // struct DriveState_

typedef ::gator_msgs::DriveState_<std::allocator<void> > DriveState;

typedef boost::shared_ptr< ::gator_msgs::DriveState > DriveStatePtr;
typedef boost::shared_ptr< ::gator_msgs::DriveState const> DriveStateConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::gator_msgs::DriveState_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::gator_msgs::DriveState_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace gator_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'gator_msgs': ['/home/reggert/Documents/GatorROS/GatorResearch/master_ws/src/gator_msgs/msg'], 'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg'], 'sensor_msgs': ['/opt/ros/indigo/share/sensor_msgs/cmake/../msg'], 'diagnostic_msgs': ['/opt/ros/indigo/share/diagnostic_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::gator_msgs::DriveState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::gator_msgs::DriveState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::gator_msgs::DriveState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::gator_msgs::DriveState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::gator_msgs::DriveState_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::gator_msgs::DriveState_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::gator_msgs::DriveState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e9add9a38f3f6f218376f584f3063c5b";
  }

  static const char* value(const ::gator_msgs::DriveState_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe9add9a38f3f6f21ULL;
  static const uint64_t static_value2 = 0x8376f584f3063c5bULL;
};

template<class ContainerAllocator>
struct DataType< ::gator_msgs::DriveState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "gator_msgs/DriveState";
  }

  static const char* value(const ::gator_msgs::DriveState_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::gator_msgs::DriveState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 steeringangle\n\
float32 wheelvelocity\n\
bool    brake\n\
";
  }

  static const char* value(const ::gator_msgs::DriveState_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::gator_msgs::DriveState_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.steeringangle);
      stream.next(m.wheelvelocity);
      stream.next(m.brake);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct DriveState_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::gator_msgs::DriveState_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::gator_msgs::DriveState_<ContainerAllocator>& v)
  {
    s << indent << "steeringangle: ";
    Printer<float>::stream(s, indent + "  ", v.steeringangle);
    s << indent << "wheelvelocity: ";
    Printer<float>::stream(s, indent + "  ", v.wheelvelocity);
    s << indent << "brake: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.brake);
  }
};

} // namespace message_operations
} // namespace ros

#endif // GATOR_MSGS_MESSAGE_DRIVESTATE_H
