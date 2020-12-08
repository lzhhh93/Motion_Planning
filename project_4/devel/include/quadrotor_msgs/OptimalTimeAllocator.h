// Generated by gencpp from file quadrotor_msgs/OptimalTimeAllocator.msg
// DO NOT EDIT!


#ifndef QUADROTOR_MSGS_MESSAGE_OPTIMALTIMEALLOCATOR_H
#define QUADROTOR_MSGS_MESSAGE_OPTIMALTIMEALLOCATOR_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace quadrotor_msgs
{
template <class ContainerAllocator>
struct OptimalTimeAllocator_
{
  typedef OptimalTimeAllocator_<ContainerAllocator> Type;

  OptimalTimeAllocator_()
    : header()
    , start_time()
    , final_time()
    , trajectory_id(0)
    , action(0)
    , K()
    , K_max(0)
    , a()
    , b()
    , s()
    , time()
    , time_acc()
    , s_step(0.0)
    , debug_info()  {
    }
  OptimalTimeAllocator_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , start_time()
    , final_time()
    , trajectory_id(0)
    , action(0)
    , K(_alloc)
    , K_max(0)
    , a(_alloc)
    , b(_alloc)
    , s(_alloc)
    , time(_alloc)
    , time_acc(_alloc)
    , s_step(0.0)
    , debug_info(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef ros::Time _start_time_type;
  _start_time_type start_time;

   typedef ros::Time _final_time_type;
  _final_time_type final_time;

   typedef uint32_t _trajectory_id_type;
  _trajectory_id_type trajectory_id;

   typedef uint32_t _action_type;
  _action_type action;

   typedef std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other >  _K_type;
  _K_type K;

   typedef int32_t _K_max_type;
  _K_max_type K_max;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _a_type;
  _a_type a;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _b_type;
  _b_type b;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _s_type;
  _s_type s;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _time_type;
  _time_type time;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _time_acc_type;
  _time_acc_type time_acc;

   typedef double _s_step_type;
  _s_step_type s_step;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _debug_info_type;
  _debug_info_type debug_info;



  enum {
    ACTION_ADD = 1u,
    ACTION_ABORT = 2u,
    ACTION_WARN_START = 3u,
    ACTION_WARN_FINAL = 4u,
    ACTION_WARN_IMPOSSIBLE = 5u,
  };


  typedef boost::shared_ptr< ::quadrotor_msgs::OptimalTimeAllocator_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::quadrotor_msgs::OptimalTimeAllocator_<ContainerAllocator> const> ConstPtr;

}; // struct OptimalTimeAllocator_

typedef ::quadrotor_msgs::OptimalTimeAllocator_<std::allocator<void> > OptimalTimeAllocator;

typedef boost::shared_ptr< ::quadrotor_msgs::OptimalTimeAllocator > OptimalTimeAllocatorPtr;
typedef boost::shared_ptr< ::quadrotor_msgs::OptimalTimeAllocator const> OptimalTimeAllocatorConstPtr;

// constants requiring out of line definition

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::quadrotor_msgs::OptimalTimeAllocator_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::quadrotor_msgs::OptimalTimeAllocator_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::quadrotor_msgs::OptimalTimeAllocator_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::quadrotor_msgs::OptimalTimeAllocator_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::quadrotor_msgs::OptimalTimeAllocator_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::quadrotor_msgs::OptimalTimeAllocator_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::quadrotor_msgs::OptimalTimeAllocator_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::quadrotor_msgs::OptimalTimeAllocator_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::quadrotor_msgs::OptimalTimeAllocator_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a6b8609f5d139106ef66e171c3643730";
  }

  static const char* value(const ::quadrotor_msgs::OptimalTimeAllocator_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa6b8609f5d139106ULL;
  static const uint64_t static_value2 = 0xef66e171c3643730ULL;
};

template<class ContainerAllocator>
struct DataType< ::quadrotor_msgs::OptimalTimeAllocator_<ContainerAllocator> >
{
  static const char* value()
  {
    return "quadrotor_msgs/OptimalTimeAllocator";
  }

  static const char* value(const ::quadrotor_msgs::OptimalTimeAllocator_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::quadrotor_msgs::OptimalTimeAllocator_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
\n\
time start_time\n\
time final_time\n\
\n\
# the trajectory id, starts from \"1\".\n\
uint32 trajectory_id\n\
\n\
# the action command for trajectory server.\n\
uint32 ACTION_ADD                  =   1\n\
uint32 ACTION_ABORT                =   2\n\
uint32 ACTION_WARN_START           =   3\n\
uint32 ACTION_WARN_FINAL           =   4\n\
uint32 ACTION_WARN_IMPOSSIBLE      =   5\n\
uint32 action\n\
\n\
# the vector of all 'K' number of each piece of the time profile.\n\
int32[] K\n\
int32   K_max\n\
\n\
# the a, b, c, d parameters of the TOPP time profile.\n\
float64[] a\n\
float64[] b\n\
\n\
# useful variables for evaluating time\n\
float64[] s\n\
float64[] time\n\
float64[] time_acc\n\
\n\
# delta_s in s domain\n\
float64   s_step\n\
\n\
string debug_info\n\
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
";
  }

  static const char* value(const ::quadrotor_msgs::OptimalTimeAllocator_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::quadrotor_msgs::OptimalTimeAllocator_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.start_time);
      stream.next(m.final_time);
      stream.next(m.trajectory_id);
      stream.next(m.action);
      stream.next(m.K);
      stream.next(m.K_max);
      stream.next(m.a);
      stream.next(m.b);
      stream.next(m.s);
      stream.next(m.time);
      stream.next(m.time_acc);
      stream.next(m.s_step);
      stream.next(m.debug_info);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct OptimalTimeAllocator_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::quadrotor_msgs::OptimalTimeAllocator_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::quadrotor_msgs::OptimalTimeAllocator_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "start_time: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.start_time);
    s << indent << "final_time: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.final_time);
    s << indent << "trajectory_id: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.trajectory_id);
    s << indent << "action: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.action);
    s << indent << "K[]" << std::endl;
    for (size_t i = 0; i < v.K.size(); ++i)
    {
      s << indent << "  K[" << i << "]: ";
      Printer<int32_t>::stream(s, indent + "  ", v.K[i]);
    }
    s << indent << "K_max: ";
    Printer<int32_t>::stream(s, indent + "  ", v.K_max);
    s << indent << "a[]" << std::endl;
    for (size_t i = 0; i < v.a.size(); ++i)
    {
      s << indent << "  a[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.a[i]);
    }
    s << indent << "b[]" << std::endl;
    for (size_t i = 0; i < v.b.size(); ++i)
    {
      s << indent << "  b[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.b[i]);
    }
    s << indent << "s[]" << std::endl;
    for (size_t i = 0; i < v.s.size(); ++i)
    {
      s << indent << "  s[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.s[i]);
    }
    s << indent << "time[]" << std::endl;
    for (size_t i = 0; i < v.time.size(); ++i)
    {
      s << indent << "  time[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.time[i]);
    }
    s << indent << "time_acc[]" << std::endl;
    for (size_t i = 0; i < v.time_acc.size(); ++i)
    {
      s << indent << "  time_acc[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.time_acc[i]);
    }
    s << indent << "s_step: ";
    Printer<double>::stream(s, indent + "  ", v.s_step);
    s << indent << "debug_info: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.debug_info);
  }
};

} // namespace message_operations
} // namespace ros

#endif // QUADROTOR_MSGS_MESSAGE_OPTIMALTIMEALLOCATOR_H
