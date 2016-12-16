// Generated by gencpp from file manipulation/solution.msg
// DO NOT EDIT!


#ifndef MANIPULATION_MESSAGE_SOLUTION_H
#define MANIPULATION_MESSAGE_SOLUTION_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace manipulation
{
template <class ContainerAllocator>
struct solution_
{
  typedef solution_<ContainerAllocator> Type;

  solution_()
    : number(0)
    , marker()
    , radian(0.0)  {
    }
  solution_(const ContainerAllocator& _alloc)
    : number(0)
    , marker(_alloc)
    , radian(0.0)  {
  (void)_alloc;
    }



   typedef int16_t _number_type;
  _number_type number;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _marker_type;
  _marker_type marker;

   typedef float _radian_type;
  _radian_type radian;




  typedef boost::shared_ptr< ::manipulation::solution_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::manipulation::solution_<ContainerAllocator> const> ConstPtr;

}; // struct solution_

typedef ::manipulation::solution_<std::allocator<void> > solution;

typedef boost::shared_ptr< ::manipulation::solution > solutionPtr;
typedef boost::shared_ptr< ::manipulation::solution const> solutionConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::manipulation::solution_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::manipulation::solution_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace manipulation

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'manipulation': ['/home/team4/ros_workspace/project/src/manipulation/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::manipulation::solution_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::manipulation::solution_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::manipulation::solution_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::manipulation::solution_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::manipulation::solution_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::manipulation::solution_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::manipulation::solution_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a6058250da7582c0420253c404ea225d";
  }

  static const char* value(const ::manipulation::solution_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa6058250da7582c0ULL;
  static const uint64_t static_value2 = 0x420253c404ea225dULL;
};

template<class ContainerAllocator>
struct DataType< ::manipulation::solution_<ContainerAllocator> >
{
  static const char* value()
  {
    return "manipulation/solution";
  }

  static const char* value(const ::manipulation::solution_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::manipulation::solution_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int16 number\n\
string marker\n\
float32 radian\n\
";
  }

  static const char* value(const ::manipulation::solution_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::manipulation::solution_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.number);
      stream.next(m.marker);
      stream.next(m.radian);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct solution_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::manipulation::solution_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::manipulation::solution_<ContainerAllocator>& v)
  {
    s << indent << "number: ";
    Printer<int16_t>::stream(s, indent + "  ", v.number);
    s << indent << "marker: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.marker);
    s << indent << "radian: ";
    Printer<float>::stream(s, indent + "  ", v.radian);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MANIPULATION_MESSAGE_SOLUTION_H