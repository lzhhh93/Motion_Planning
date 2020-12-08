// Generated by gencpp from file quadrotor_msgs/PositionCommand.msg
// DO NOT EDIT!


#ifndef QUADROTOR_MSGS_MESSAGE_POSITIONCOMMAND_H
#define QUADROTOR_MSGS_MESSAGE_POSITIONCOMMAND_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3.h>

namespace quadrotor_msgs
{
template <class ContainerAllocator>
struct PositionCommand_
{
  typedef PositionCommand_<ContainerAllocator> Type;

  PositionCommand_()
    : header()
    , position()
    , velocity()
    , acceleration()
    , yaw(0.0)
    , yaw_dot(0.0)
    , kx()
    , kv()
    , trajectory_id(0)
    , trajectory_flag(0)  {
      kx.assign(0.0);

      kv.assign(0.0);
  }
  PositionCommand_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , position(_alloc)
    , velocity(_alloc)
    , acceleration(_alloc)
    , yaw(0.0)
    , yaw_dot(0.0)
    , kx()
    , kv()
    , trajectory_id(0)
    , trajectory_flag(0)  {
  (void)_alloc;
      kx.assign(0.0);

      kv.assign(0.0);
  }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _position_type;
  _position_type position;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _velocity_type;
  _velocity_type velocity;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _acceleration_type;
  _acceleration_type acceleration;

   typedef double _yaw_type;
  _yaw_type yaw;

   typedef double _yaw_dot_type;
  _yaw_dot_type yaw_dot;

   typedef boost::array<double, 3>  _kx_type;
  _kx_type kx;

   typedef boost::array<double, 3>  _kv_type;
  _kv_type kv;

   typedef uint32_t _trajectory_id_type;
  _trajectory_id_type trajectory_id;

   typedef uint8_t _trajectory_flag_type;
  _trajectory_flag_type trajectory_flag;



  enum {
    TRAJECTORY_STATUS_EMPTY = 0u,
    TRAJECTORY_STATUS_READY = 1u,
    TRAJECTORY_STATUS_COMPLETED = 3u,
    TRAJECTROY_STATUS_ABORT = 4u,
    TRAJECTORY_STATUS_ILLEGAL_START = 5u,
    TRAJECTORY_STATUS_ILLEGAL_FINAL = 6u,
    TRAJECTORY_STATUS_IMPOSSIBLE = 7u,
  };


  typedef boost::shared_ptr< ::quadrotor_msgs::PositionCommand_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::quadrotor_msgs::PositionCommand_<ContainerAllocator> const> ConstPtr;

}; // struct PositionCommand_

typedef ::quadrotor_msgs::PositionCommand_<std::allocator<void> > PositionCommand;

typedef boost::shared_ptr< ::quadrotor_msgs::PositionCommand > PositionCommandPtr;
typedef boost::shared_ptr< ::quadrotor_msgs::PositionCommand const> PositionCommandConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::quadrotor_msgs::PositionCommand_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::quadrotor_msgs::PositionCommand_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace quadrotor_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'nav_msgs': ['/opt/ros/kinetic/share/nav_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'quadrotor_msgs': ['/home/zhanhao/catkin_ch5/src/quadrotor_msgs/msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::quadrotor_msgs::PositionCommand_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::quadrotor_msgs::PositionCommand_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::quadrotor_msgs::PositionCommand_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::quadrotor_msgs::PositionCommand_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::quadrotor_msgs::PositionCommand_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::quadrotor_msgs::PositionCommand_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::quadrotor_msgs::PositionCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4712f0609ca29a79af79a35ca3e3967a";
  }

  static const char* value(const ::quadrotor_msgs::PositionCommand_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4712f0609ca29a79ULL;
  static const uint64_t static_value2 = 0xaf79a35ca3e3967aULL;
};

template<class ContainerAllocator>
struct DataType< ::quadrotor_msgs::PositionCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "quadrotor_msgs/PositionCommand";
  }

  static const char* value(const ::quadrotor_msgs::PositionCommand_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::quadrotor_msgs::PositionCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
geometry_msgs/Point position\n\
geometry_msgs/Vector3 velocity\n\
geometry_msgs/Vector3 acceleration\n\
float64 yaw\n\
float64 yaw_dot\n\
float64[3] kx\n\
float64[3] kv \n\
\n\
uint32 trajectory_id\n\
\n\
uint8 TRAJECTORY_STATUS_EMPTY = 0\n\
uint8 TRAJECTORY_STATUS_READY = 1\n\
uint8 TRAJECTORY_STATUS_COMPLETED = 3\n\
uint8 TRAJECTROY_STATUS_ABORT = 4\n\
uint8 TRAJECTORY_STATUS_ILLEGAL_START = 5\n\
uint8 TRAJECTORY_STATUS_ILLEGAL_FINAL = 6\n\
uint8 TRAJECTORY_STATUS_IMPOSSIBLE = 7\n\
\n\
# Its ID number will start from 1, allowing you comparing it with 0.\n\
uint8 trajectory_flag\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Vector3\n\
# This represents a vector in free space. \n\
# It is only meant to represent a direction. Therefore, it does not\n\
# make sense to apply a translation to it (e.g., when applying a \n\
# generic rigid transformation to a Vector3, tf2 will only apply the\n\
# rotation). If you want your data to be translatable too, use the\n\
# geometry_msgs/Point message instead.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
";
  }

  static const char* value(const ::quadrotor_msgs::PositionCommand_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::quadrotor_msgs::PositionCommand_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.position);
      stream.next(m.velocity);
      stream.next(m.acceleration);
      stream.next(m.yaw);
      stream.next(m.yaw_dot);
      stream.next(m.kx);
      stream.next(m.kv);
      stream.next(m.trajectory_id);
      stream.next(m.trajectory_flag);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PositionCommand_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::quadrotor_msgs::PositionCommand_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::quadrotor_msgs::PositionCommand_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "position: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.position);
    s << indent << "velocity: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.velocity);
    s << indent << "acceleration: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.acceleration);
    s << indent << "yaw: ";
    Printer<double>::stream(s, indent + "  ", v.yaw);
    s << indent << "yaw_dot: ";
    Printer<double>::stream(s, indent + "  ", v.yaw_dot);
    s << indent << "kx[]" << std::endl;
    for (size_t i = 0; i < v.kx.size(); ++i)
    {
      s << indent << "  kx[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.kx[i]);
    }
    s << indent << "kv[]" << std::endl;
    for (size_t i = 0; i < v.kv.size(); ++i)
    {
      s << indent << "  kv[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.kv[i]);
    }
    s << indent << "trajectory_id: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.trajectory_id);
    s << indent << "trajectory_flag: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.trajectory_flag);
  }
};

} // namespace message_operations
} // namespace ros

#endif // QUADROTOR_MSGS_MESSAGE_POSITIONCOMMAND_H
