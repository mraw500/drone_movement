/* Auto-generated by genmsg_cpp for file /home/robotlab/fuerte_workspace/tum_ardrone/msg/filter_state.msg */
#ifndef TUM_ARDRONE_MESSAGE_FILTER_STATE_H
#define TUM_ARDRONE_MESSAGE_FILTER_STATE_H
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

#include "std_msgs/Header.h"

namespace tum_ardrone
{
template <class ContainerAllocator>
struct filter_state_ {
  typedef filter_state_<ContainerAllocator> Type;

  filter_state_()
  : header()
  , x(0.0)
  , y(0.0)
  , z(0.0)
  , dx(0.0)
  , dy(0.0)
  , dz(0.0)
  , roll(0.0)
  , pitch(0.0)
  , yaw(0.0)
  , dyaw(0.0)
  , scale(0.0)
  , ptamState(0)
  , scaleAccuracy(0.0)
  , droneState(0)
  , batteryPercent(0.0)
  {
  }

  filter_state_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , x(0.0)
  , y(0.0)
  , z(0.0)
  , dx(0.0)
  , dy(0.0)
  , dz(0.0)
  , roll(0.0)
  , pitch(0.0)
  , yaw(0.0)
  , dyaw(0.0)
  , scale(0.0)
  , ptamState(0)
  , scaleAccuracy(0.0)
  , droneState(0)
  , batteryPercent(0.0)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef float _x_type;
  float x;

  typedef float _y_type;
  float y;

  typedef float _z_type;
  float z;

  typedef float _dx_type;
  float dx;

  typedef float _dy_type;
  float dy;

  typedef float _dz_type;
  float dz;

  typedef float _roll_type;
  float roll;

  typedef float _pitch_type;
  float pitch;

  typedef float _yaw_type;
  float yaw;

  typedef float _dyaw_type;
  float dyaw;

  typedef float _scale_type;
  float scale;

  typedef uint32_t _ptamState_type;
  uint32_t ptamState;

  typedef float _scaleAccuracy_type;
  float scaleAccuracy;

  typedef uint32_t _droneState_type;
  uint32_t droneState;

  typedef float _batteryPercent_type;
  float batteryPercent;

  enum { PTAM_IDLE = 0 };
  enum { PTAM_INITIALIZING = 1 };
  enum { PTAM_LOST = 2 };
  enum { PTAM_GOOD = 3 };
  enum { PTAM_BEST = 4 };
  enum { PTAM_TOOKKF = 5 };
  enum { PTAM_FALSEPOSITIVE = 6 };

  typedef boost::shared_ptr< ::tum_ardrone::filter_state_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::tum_ardrone::filter_state_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct filter_state
typedef  ::tum_ardrone::filter_state_<std::allocator<void> > filter_state;

typedef boost::shared_ptr< ::tum_ardrone::filter_state> filter_statePtr;
typedef boost::shared_ptr< ::tum_ardrone::filter_state const> filter_stateConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::tum_ardrone::filter_state_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::tum_ardrone::filter_state_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace tum_ardrone

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::tum_ardrone::filter_state_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::tum_ardrone::filter_state_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::tum_ardrone::filter_state_<ContainerAllocator> > {
  static const char* value() 
  {
    return "33f8050d082c4ebadff26def43dcfd15";
  }

  static const char* value(const  ::tum_ardrone::filter_state_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x33f8050d082c4ebaULL;
  static const uint64_t static_value2 = 0xdff26def43dcfd15ULL;
};

template<class ContainerAllocator>
struct DataType< ::tum_ardrone::filter_state_<ContainerAllocator> > {
  static const char* value() 
  {
    return "tum_ardrone/filter_state";
  }

  static const char* value(const  ::tum_ardrone::filter_state_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::tum_ardrone::filter_state_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# constants\n\
uint32 PTAM_IDLE = 0           # PTAM not running.\n\
uint32 PTAM_INITIALIZING = 1   # initialization (trails)\n\
uint32 PTAM_LOST = 2           # ptam is running, but lost\n\
uint32 PTAM_GOOD = 3           # tracking quality OK\n\
uint32 PTAM_BEST = 4           # tracking quality best\n\
uint32 PTAM_TOOKKF = 5         # just took a new KF (equivalent to PTAM_BEST)\n\
uint32 PTAM_FALSEPOSITIVE = 6  # ptam thinks it is good, but its estimate is rejected.\n\
\n\
# header\n\
Header      header\n\
\n\
# ----------------- raw 10d filter state ----------------------------\n\
float32     x\n\
float32     y\n\
float32     z\n\
float32     dx\n\
float32     dy\n\
float32     dz\n\
float32     roll\n\
float32     pitch\n\
float32     yaw\n\
float32     dyaw\n\
\n\
\n\
\n\
# --------------------- other values ---------------------------\n\
float32     scale         # ptam scale factor (PTAMpos * scale = WORLDpos).\n\
uint32      ptamState\n\
float32     scaleAccuracy # if scale is very inaccurate, this is about 0.5, and grows up to 1 (=very accurate).\n\
\n\
\n\
# ----------------- propagated from drone messages: -----------------\n\
# 0: Unknown, 1: Init, 2: Landed, 3: Flying, 4: Hovering, 5: Test\n\
# 6: Taking off, 7: Goto Fix Point, 8: Landing, 9: Looping\n\
# Note: 3,7 seems to discriminate type of flying (isFly = 3 | 7)\n\
uint32       droneState\n\
float32      batteryPercent    # 0 means no battery, 100 means full battery\n\
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
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
";
  }

  static const char* value(const  ::tum_ardrone::filter_state_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::tum_ardrone::filter_state_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::tum_ardrone::filter_state_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::tum_ardrone::filter_state_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.x);
    stream.next(m.y);
    stream.next(m.z);
    stream.next(m.dx);
    stream.next(m.dy);
    stream.next(m.dz);
    stream.next(m.roll);
    stream.next(m.pitch);
    stream.next(m.yaw);
    stream.next(m.dyaw);
    stream.next(m.scale);
    stream.next(m.ptamState);
    stream.next(m.scaleAccuracy);
    stream.next(m.droneState);
    stream.next(m.batteryPercent);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct filter_state_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::tum_ardrone::filter_state_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::tum_ardrone::filter_state_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "x: ";
    Printer<float>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<float>::stream(s, indent + "  ", v.y);
    s << indent << "z: ";
    Printer<float>::stream(s, indent + "  ", v.z);
    s << indent << "dx: ";
    Printer<float>::stream(s, indent + "  ", v.dx);
    s << indent << "dy: ";
    Printer<float>::stream(s, indent + "  ", v.dy);
    s << indent << "dz: ";
    Printer<float>::stream(s, indent + "  ", v.dz);
    s << indent << "roll: ";
    Printer<float>::stream(s, indent + "  ", v.roll);
    s << indent << "pitch: ";
    Printer<float>::stream(s, indent + "  ", v.pitch);
    s << indent << "yaw: ";
    Printer<float>::stream(s, indent + "  ", v.yaw);
    s << indent << "dyaw: ";
    Printer<float>::stream(s, indent + "  ", v.dyaw);
    s << indent << "scale: ";
    Printer<float>::stream(s, indent + "  ", v.scale);
    s << indent << "ptamState: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.ptamState);
    s << indent << "scaleAccuracy: ";
    Printer<float>::stream(s, indent + "  ", v.scaleAccuracy);
    s << indent << "droneState: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.droneState);
    s << indent << "batteryPercent: ";
    Printer<float>::stream(s, indent + "  ", v.batteryPercent);
  }
};


} // namespace message_operations
} // namespace ros

#endif // TUM_ARDRONE_MESSAGE_FILTER_STATE_H

