// Generated by gencpp from file swipe_obstacles/detected_obstacle_array.msg
// DO NOT EDIT!


#ifndef SWIPE_OBSTACLES_MESSAGE_DETECTED_OBSTACLE_ARRAY_H
#define SWIPE_OBSTACLES_MESSAGE_DETECTED_OBSTACLE_ARRAY_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <swipe_obstacles/detected_obstacle.h>

namespace swipe_obstacles
{
template <class ContainerAllocator>
struct detected_obstacle_array_
{
  typedef detected_obstacle_array_<ContainerAllocator> Type;

  detected_obstacle_array_()
    : header()
    , obstacles()  {
    }
  detected_obstacle_array_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , obstacles(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector< ::swipe_obstacles::detected_obstacle_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::swipe_obstacles::detected_obstacle_<ContainerAllocator> >::other >  _obstacles_type;
  _obstacles_type obstacles;





  typedef boost::shared_ptr< ::swipe_obstacles::detected_obstacle_array_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::swipe_obstacles::detected_obstacle_array_<ContainerAllocator> const> ConstPtr;

}; // struct detected_obstacle_array_

typedef ::swipe_obstacles::detected_obstacle_array_<std::allocator<void> > detected_obstacle_array;

typedef boost::shared_ptr< ::swipe_obstacles::detected_obstacle_array > detected_obstacle_arrayPtr;
typedef boost::shared_ptr< ::swipe_obstacles::detected_obstacle_array const> detected_obstacle_arrayConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::swipe_obstacles::detected_obstacle_array_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::swipe_obstacles::detected_obstacle_array_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace swipe_obstacles

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'pcl_msgs': ['/opt/ros/kinetic/share/pcl_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'swipe_obstacles': ['/home/kuriatsu/Program/Ros/master_study_ws/src/swipe_obstacles/msg'], 'jsk_footstep_msgs': ['/opt/ros/kinetic/share/jsk_footstep_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'visualization_msgs': ['/opt/ros/kinetic/share/visualization_msgs/cmake/../msg'], 'jsk_recognition_msgs': ['/opt/ros/kinetic/share/jsk_recognition_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg'], 'autoware_msgs': ['/home/kuriatsu/Autoware/ros/src/msgs/autoware_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::swipe_obstacles::detected_obstacle_array_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::swipe_obstacles::detected_obstacle_array_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::swipe_obstacles::detected_obstacle_array_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::swipe_obstacles::detected_obstacle_array_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::swipe_obstacles::detected_obstacle_array_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::swipe_obstacles::detected_obstacle_array_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::swipe_obstacles::detected_obstacle_array_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e900cf0096e79af6967f71da5a89cfd0";
  }

  static const char* value(const ::swipe_obstacles::detected_obstacle_array_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe900cf0096e79af6ULL;
  static const uint64_t static_value2 = 0x967f71da5a89cfd0ULL;
};

template<class ContainerAllocator>
struct DataType< ::swipe_obstacles::detected_obstacle_array_<ContainerAllocator> >
{
  static const char* value()
  {
    return "swipe_obstacles/detected_obstacle_array";
  }

  static const char* value(const ::swipe_obstacles::detected_obstacle_array_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::swipe_obstacles::detected_obstacle_array_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header header\n\
detected_obstacle[] obstacles\n\
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
MSG: swipe_obstacles/detected_obstacle\n\
std_msgs/Header header\n\
\n\
uint32 id\n\
uint32 managed_id\n\
string label\n\
float32 score\n\
geometry_msgs/Pose pose\n\
\n\
float32 shift_x\n\
float32 shift_y\n\
uint32 visible\n\
time detected_time\n\
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

  static const char* value(const ::swipe_obstacles::detected_obstacle_array_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::swipe_obstacles::detected_obstacle_array_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.obstacles);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct detected_obstacle_array_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::swipe_obstacles::detected_obstacle_array_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::swipe_obstacles::detected_obstacle_array_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "obstacles[]" << std::endl;
    for (size_t i = 0; i < v.obstacles.size(); ++i)
    {
      s << indent << "  obstacles[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::swipe_obstacles::detected_obstacle_<ContainerAllocator> >::stream(s, indent + "    ", v.obstacles[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // SWIPE_OBSTACLES_MESSAGE_DETECTED_OBSTACLE_ARRAY_H
