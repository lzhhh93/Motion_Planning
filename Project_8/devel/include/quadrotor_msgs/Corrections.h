// Generated by gencpp from file quadrotor_msgs/Corrections.msg
// DO NOT EDIT!


#ifndef QUADROTOR_MSGS_MESSAGE_CORRECTIONS_H
#define QUADROTOR_MSGS_MESSAGE_CORRECTIONS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace quadrotor_msgs
{
template <class ContainerAllocator>
struct Corrections_
{
  typedef Corrections_<ContainerAllocator> Type;

  Corrections_()
    : kf_correction(0.0)
    , angle_corrections()  {
      angle_corrections.assign(0.0);
  }
  Corrections_(const ContainerAllocator& _alloc)
    : kf_correction(0.0)
    , angle_corrections()  {
  (void)_alloc;
      angle_corrections.assign(0.0);
  }



   typedef double _kf_correction_type;
  _kf_correction_type kf_correction;

   typedef boost::array<double, 2>  _angle_corrections_type;
  _angle_corrections_type angle_corrections;





  typedef boost::shared_ptr< ::quadrotor_msgs::Corrections_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::quadrotor_msgs::Corrections_<ContainerAllocator> const> ConstPtr;

}; // struct Corrections_

typedef ::quadrotor_msgs::Corrections_<std::allocator<void> > Corrections;

typedef boost::shared_ptr< ::quadrotor_msgs::Corrections > CorrectionsPtr;
typedef boost::shared_ptr< ::quadrotor_msgs::Corrections const> CorrectionsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::quadrotor_msgs::Corrections_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::quadrotor_msgs::Corrections_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace quadrotor_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'nav_msgs': ['/opt/ros/kinetic/share/nav_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'quadrotor_msgs': ['/home/zhanhao/Project/src/__MACOSX/src/read_only/Utils/quadrotor_msgs/msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::quadrotor_msgs::Corrections_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::quadrotor_msgs::Corrections_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::quadrotor_msgs::Corrections_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::quadrotor_msgs::Corrections_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::quadrotor_msgs::Corrections_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::quadrotor_msgs::Corrections_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::quadrotor_msgs::Corrections_<ContainerAllocator> >
{
  static const char* value()
  {
    return "61e86887a75fe520847d3256306360f5";
  }

  static const char* value(const ::quadrotor_msgs::Corrections_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x61e86887a75fe520ULL;
  static const uint64_t static_value2 = 0x847d3256306360f5ULL;
};

template<class ContainerAllocator>
struct DataType< ::quadrotor_msgs::Corrections_<ContainerAllocator> >
{
  static const char* value()
  {
    return "quadrotor_msgs/Corrections";
  }

  static const char* value(const ::quadrotor_msgs::Corrections_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::quadrotor_msgs::Corrections_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 kf_correction\n\
float64[2] angle_corrections\n\
";
  }

  static const char* value(const ::quadrotor_msgs::Corrections_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::quadrotor_msgs::Corrections_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.kf_correction);
      stream.next(m.angle_corrections);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Corrections_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::quadrotor_msgs::Corrections_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::quadrotor_msgs::Corrections_<ContainerAllocator>& v)
  {
    s << indent << "kf_correction: ";
    Printer<double>::stream(s, indent + "  ", v.kf_correction);
    s << indent << "angle_corrections[]" << std::endl;
    for (size_t i = 0; i < v.angle_corrections.size(); ++i)
    {
      s << indent << "  angle_corrections[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.angle_corrections[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // QUADROTOR_MSGS_MESSAGE_CORRECTIONS_H
