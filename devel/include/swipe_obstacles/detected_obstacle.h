// Generated by gencpp from file swipe_obstacles/detected_obstacle.msg
// DO NOT EDIT!


#ifndef SWIPE_OBSTACLES_MESSAGE_DETECTED_OBSTACLE_H
#define SWIPE_OBSTACLES_MESSAGE_DETECTED_OBSTACLE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/Pose.h>

namespace swipe_obstacles
{
template <class ContainerAllocator>
struct detected_obstacle_
{
  typedef detected_obstacle_<ContainerAllocator> Type;

  detected_obstacle_()
    : header()
    , id(0)
    , managed_id(0)
    , label()
    , score(0.0)
    , pose()
    , shift_x(0.0)
    , shift_y(0.0)
    , round(0)
    , detected_time()  {
    }
  detected_obstacle_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , id(0)
    , managed_id(0)
    , label(_alloc)
    , score(0.0)
    , pose(_alloc)
    , shift_x(0.0)
    , shift_y(0.0)
    , round(0)
    , detected_time()  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint32_t _id_type;
  _id_type id;

   typedef uint32_t _managed_id_type;
  _managed_id_type managed_id;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _label_type;
  _label_type label;

   typedef float _score_type;
  _score_type score;

   typedef  ::geometry_msgs::Pose_<ContainerAllocator>  _pose_type;
  _pose_type pose;

   typedef float _shift_x_type;
  _shift_x_type shift_x;

   typedef float _shift_y_type;
  _shift_y_type shift_y;

   typedef uint32_t _round_type;
  _round_type round;

   typedef ros::Time _detected_time_type;
  _detected_time_type detected_time;





  typedef boost::shared_ptr< ::swipe_obstacles::detected_obstacle_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::swipe_obstacles::detected_obstacle_<ContainerAllocator> const> ConstPtr;

}; // struct detected_obstacle_

typedef ::swipe_obstacles::detected_obstacle_<std::allocator<void> > detected_obstacle;

typedef boost::shared_ptr< ::swipe_obstacles::detected_obstacle > detected_obstaclePtr;
typedef boost::shared_ptr< ::swipe_obstacles::detected_obstacle const> detected_obstacleConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::swipe_obstacles::detected_obstacle_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::swipe_obstacles::detected_obstacle_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace swipe_obstacles

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'swipe_obstacles': ['/home/kuriatsu/Program/Ros/master_study_ws/src/swipe_obstacles/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::swipe_obstacles::detected_obstacle_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::swipe_obstacles::detected_obstacle_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::swipe_obstacles::detected_obstacle_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::swipe_obstacles::detected_obstacle_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::swipe_obstacles::detected_obstacle_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::swipe_obstacles::detected_obstacle_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::swipe_obstacles::detected_obstacle_<ContainerAllocator> >
{
  static const char* value()
  {
    return "9978e5bb58ee01f8e45fc2d8376b759a";
  }

  static const char* value(const ::swipe_obstacles::detected_obstacle_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x9978e5bb58ee01f8ULL;
  static const uint64_t static_value2 = 0xe45fc2d8376b759aULL;
};

template<class ContainerAllocator>
struct DataType< ::swipe_obstacles::detected_obstacle_<ContainerAllocator> >
{
  static const char* value()
  {
    return "swipe_obstacles/detected_obstacle";
  }

  static const char* value(const ::swipe_obstacles::detected_obstacle_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::swipe_obstacles::detected_obstacle_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header header\n\
\n\
uint32 id\n\
uint32 managed_id\n\
string label\n\
float32 score\n\
geometry_msgs/Pose pose\n\
\n\
float32 shift_x\n\
float32 shift_y\n\
uint32 round\n\
time detected_time\n\
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
MSG: geometry_msgs/Pose\n\
# A representation of pose in free space, composed of position and orientation. \n\
Point position\n\
Quaternion orientation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
";
  }

  static const char* value(const ::swipe_obstacles::detected_obstacle_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::swipe_obstacles::detected_obstacle_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.id);
      stream.next(m.managed_id);
      stream.next(m.label);
      stream.next(m.score);
      stream.next(m.pose);
      stream.next(m.shift_x);
      stream.next(m.shift_y);
      stream.next(m.round);
      stream.next(m.detected_time);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct detected_obstacle_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::swipe_obstacles::detected_obstacle_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::swipe_obstacles::detected_obstacle_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "id: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.id);
    s << indent << "managed_id: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.managed_id);
    s << indent << "label: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.label);
    s << indent << "score: ";
    Printer<float>::stream(s, indent + "  ", v.score);
    s << indent << "pose: ";
    s << std::endl;
    Printer< ::geometry_msgs::Pose_<ContainerAllocator> >::stream(s, indent + "  ", v.pose);
    s << indent << "shift_x: ";
    Printer<float>::stream(s, indent + "  ", v.shift_x);
    s << indent << "shift_y: ";
    Printer<float>::stream(s, indent + "  ", v.shift_y);
    s << indent << "round: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.round);
    s << indent << "detected_time: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.detected_time);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SWIPE_OBSTACLES_MESSAGE_DETECTED_OBSTACLE_H
