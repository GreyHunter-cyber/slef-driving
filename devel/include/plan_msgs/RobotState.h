// Generated by gencpp from file plan_msgs/RobotState.msg
// DO NOT EDIT!


#ifndef PLAN_MSGS_MESSAGE_ROBOTSTATE_H
#define PLAN_MSGS_MESSAGE_ROBOTSTATE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace plan_msgs
{
template <class ContainerAllocator>
struct RobotState_
{
  typedef RobotState_<ContainerAllocator> Type;

  RobotState_()
    : mition_arrived(false)
    , mition_arrive_num(0)
    , Speed(0.0)
    , Azimuth(0.0)
    , Stop(false)
    , loc_fix(0)  {
    }
  RobotState_(const ContainerAllocator& _alloc)
    : mition_arrived(false)
    , mition_arrive_num(0)
    , Speed(0.0)
    , Azimuth(0.0)
    , Stop(false)
    , loc_fix(0)  {
  (void)_alloc;
    }



   typedef uint8_t _mition_arrived_type;
  _mition_arrived_type mition_arrived;

   typedef int32_t _mition_arrive_num_type;
  _mition_arrive_num_type mition_arrive_num;

   typedef float _Speed_type;
  _Speed_type Speed;

   typedef float _Azimuth_type;
  _Azimuth_type Azimuth;

   typedef uint8_t _Stop_type;
  _Stop_type Stop;

   typedef int32_t _loc_fix_type;
  _loc_fix_type loc_fix;





  typedef boost::shared_ptr< ::plan_msgs::RobotState_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::plan_msgs::RobotState_<ContainerAllocator> const> ConstPtr;

}; // struct RobotState_

typedef ::plan_msgs::RobotState_<std::allocator<void> > RobotState;

typedef boost::shared_ptr< ::plan_msgs::RobotState > RobotStatePtr;
typedef boost::shared_ptr< ::plan_msgs::RobotState const> RobotStateConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::plan_msgs::RobotState_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::plan_msgs::RobotState_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::plan_msgs::RobotState_<ContainerAllocator1> & lhs, const ::plan_msgs::RobotState_<ContainerAllocator2> & rhs)
{
  return lhs.mition_arrived == rhs.mition_arrived &&
    lhs.mition_arrive_num == rhs.mition_arrive_num &&
    lhs.Speed == rhs.Speed &&
    lhs.Azimuth == rhs.Azimuth &&
    lhs.Stop == rhs.Stop &&
    lhs.loc_fix == rhs.loc_fix;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::plan_msgs::RobotState_<ContainerAllocator1> & lhs, const ::plan_msgs::RobotState_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace plan_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::plan_msgs::RobotState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::plan_msgs::RobotState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::plan_msgs::RobotState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::plan_msgs::RobotState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::plan_msgs::RobotState_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::plan_msgs::RobotState_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::plan_msgs::RobotState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ab96e98e2f0ba56ad199d5f5d6baa6be";
  }

  static const char* value(const ::plan_msgs::RobotState_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xab96e98e2f0ba56aULL;
  static const uint64_t static_value2 = 0xd199d5f5d6baa6beULL;
};

template<class ContainerAllocator>
struct DataType< ::plan_msgs::RobotState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "plan_msgs/RobotState";
  }

  static const char* value(const ::plan_msgs::RobotState_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::plan_msgs::RobotState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "#topic: /RobotState\n"
"bool mition_arrived # arrived\n"
"int32 mition_arrive_num # arrived whitch misstion point\n"
"float32 Speed\n"
"float32 Azimuth\n"
"bool  Stop     # pause or stop flag enable\n"
"int32 loc_fix  # Positioning quality 0: miss; 1: fixed; 2: float;\n"
;
  }

  static const char* value(const ::plan_msgs::RobotState_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::plan_msgs::RobotState_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.mition_arrived);
      stream.next(m.mition_arrive_num);
      stream.next(m.Speed);
      stream.next(m.Azimuth);
      stream.next(m.Stop);
      stream.next(m.loc_fix);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RobotState_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::plan_msgs::RobotState_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::plan_msgs::RobotState_<ContainerAllocator>& v)
  {
    s << indent << "mition_arrived: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.mition_arrived);
    s << indent << "mition_arrive_num: ";
    Printer<int32_t>::stream(s, indent + "  ", v.mition_arrive_num);
    s << indent << "Speed: ";
    Printer<float>::stream(s, indent + "  ", v.Speed);
    s << indent << "Azimuth: ";
    Printer<float>::stream(s, indent + "  ", v.Azimuth);
    s << indent << "Stop: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.Stop);
    s << indent << "loc_fix: ";
    Printer<int32_t>::stream(s, indent + "  ", v.loc_fix);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PLAN_MSGS_MESSAGE_ROBOTSTATE_H
