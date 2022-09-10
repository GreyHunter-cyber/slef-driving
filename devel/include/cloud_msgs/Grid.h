// Generated by gencpp from file cloud_msgs/Grid.msg
// DO NOT EDIT!


#ifndef CLOUD_MSGS_MESSAGE_GRID_H
#define CLOUD_MSGS_MESSAGE_GRID_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <cloud_msgs/PointXYA.h>

namespace cloud_msgs
{
template <class ContainerAllocator>
struct Grid_
{
  typedef Grid_<ContainerAllocator> Type;

  Grid_()
    : timestamp()
    , width(0)
    , height(0)
    , width_step(0.0)
    , height_step(0.0)
    , grid_nums(0)
    , grid()
    , enabled(0)
    , pos_vehicle()  {
    }
  Grid_(const ContainerAllocator& _alloc)
    : timestamp()
    , width(0)
    , height(0)
    , width_step(0.0)
    , height_step(0.0)
    , grid_nums(0)
    , grid(_alloc)
    , enabled(0)
    , pos_vehicle(_alloc)  {
  (void)_alloc;
    }



   typedef ros::Time _timestamp_type;
  _timestamp_type timestamp;

   typedef int32_t _width_type;
  _width_type width;

   typedef int32_t _height_type;
  _height_type height;

   typedef float _width_step_type;
  _width_step_type width_step;

   typedef float _height_step_type;
  _height_step_type height_step;

   typedef int32_t _grid_nums_type;
  _grid_nums_type grid_nums;

   typedef std::vector<int8_t, typename ContainerAllocator::template rebind<int8_t>::other >  _grid_type;
  _grid_type grid;

   typedef int8_t _enabled_type;
  _enabled_type enabled;

   typedef  ::cloud_msgs::PointXYA_<ContainerAllocator>  _pos_vehicle_type;
  _pos_vehicle_type pos_vehicle;





  typedef boost::shared_ptr< ::cloud_msgs::Grid_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cloud_msgs::Grid_<ContainerAllocator> const> ConstPtr;

}; // struct Grid_

typedef ::cloud_msgs::Grid_<std::allocator<void> > Grid;

typedef boost::shared_ptr< ::cloud_msgs::Grid > GridPtr;
typedef boost::shared_ptr< ::cloud_msgs::Grid const> GridConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::cloud_msgs::Grid_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::cloud_msgs::Grid_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::cloud_msgs::Grid_<ContainerAllocator1> & lhs, const ::cloud_msgs::Grid_<ContainerAllocator2> & rhs)
{
  return lhs.timestamp == rhs.timestamp &&
    lhs.width == rhs.width &&
    lhs.height == rhs.height &&
    lhs.width_step == rhs.width_step &&
    lhs.height_step == rhs.height_step &&
    lhs.grid_nums == rhs.grid_nums &&
    lhs.grid == rhs.grid &&
    lhs.enabled == rhs.enabled &&
    lhs.pos_vehicle == rhs.pos_vehicle;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::cloud_msgs::Grid_<ContainerAllocator1> & lhs, const ::cloud_msgs::Grid_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace cloud_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::cloud_msgs::Grid_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cloud_msgs::Grid_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cloud_msgs::Grid_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cloud_msgs::Grid_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cloud_msgs::Grid_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cloud_msgs::Grid_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::cloud_msgs::Grid_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ec789739e2a01936ea531728fbd248c4";
  }

  static const char* value(const ::cloud_msgs::Grid_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xec789739e2a01936ULL;
  static const uint64_t static_value2 = 0xea531728fbd248c4ULL;
};

template<class ContainerAllocator>
struct DataType< ::cloud_msgs::Grid_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cloud_msgs/Grid";
  }

  static const char* value(const ::cloud_msgs::Grid_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::cloud_msgs::Grid_<ContainerAllocator> >
{
  static const char* value()
  {
    return "time       timestamp\n"
"int32      width\n"
"int32      height\n"
"float32    width_step\n"
"float32    height_step\n"
"int32      grid_nums\n"
"int8[]     grid\n"
"int8       enabled\n"
"PointXYA   pos_vehicle\n"
"\n"
"================================================================================\n"
"MSG: cloud_msgs/PointXYA\n"
"float64 x\n"
"float64 y\n"
"float64 yaw  # degs\n"
;
  }

  static const char* value(const ::cloud_msgs::Grid_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::cloud_msgs::Grid_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.timestamp);
      stream.next(m.width);
      stream.next(m.height);
      stream.next(m.width_step);
      stream.next(m.height_step);
      stream.next(m.grid_nums);
      stream.next(m.grid);
      stream.next(m.enabled);
      stream.next(m.pos_vehicle);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Grid_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cloud_msgs::Grid_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::cloud_msgs::Grid_<ContainerAllocator>& v)
  {
    s << indent << "timestamp: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.timestamp);
    s << indent << "width: ";
    Printer<int32_t>::stream(s, indent + "  ", v.width);
    s << indent << "height: ";
    Printer<int32_t>::stream(s, indent + "  ", v.height);
    s << indent << "width_step: ";
    Printer<float>::stream(s, indent + "  ", v.width_step);
    s << indent << "height_step: ";
    Printer<float>::stream(s, indent + "  ", v.height_step);
    s << indent << "grid_nums: ";
    Printer<int32_t>::stream(s, indent + "  ", v.grid_nums);
    s << indent << "grid[]" << std::endl;
    for (size_t i = 0; i < v.grid.size(); ++i)
    {
      s << indent << "  grid[" << i << "]: ";
      Printer<int8_t>::stream(s, indent + "  ", v.grid[i]);
    }
    s << indent << "enabled: ";
    Printer<int8_t>::stream(s, indent + "  ", v.enabled);
    s << indent << "pos_vehicle: ";
    s << std::endl;
    Printer< ::cloud_msgs::PointXYA_<ContainerAllocator> >::stream(s, indent + "  ", v.pos_vehicle);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CLOUD_MSGS_MESSAGE_GRID_H