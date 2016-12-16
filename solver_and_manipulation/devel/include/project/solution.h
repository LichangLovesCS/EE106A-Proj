// Generated by gencpp from file project/solution.msg
// DO NOT EDIT!


#ifndef PROJECT_MESSAGE_SOLUTION_H
#define PROJECT_MESSAGE_SOLUTION_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace project
{
template <class ContainerAllocator>
struct solution_
{
  typedef solution_<ContainerAllocator> Type;

  solution_()
    : number(0)
    , face()
    , radian(0.0)  {
    }
  solution_(const ContainerAllocator& _alloc)
    : number(0)
    , face(_alloc)
    , radian(0.0)  {
  (void)_alloc;
    }



   typedef int16_t _number_type;
  _number_type number;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _face_type;
  _face_type face;

   typedef float _radian_type;
  _radian_type radian;




  typedef boost::shared_ptr< ::project::solution_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::project::solution_<ContainerAllocator> const> ConstPtr;

}; // struct solution_

typedef ::project::solution_<std::allocator<void> > solution;

typedef boost::shared_ptr< ::project::solution > solutionPtr;
typedef boost::shared_ptr< ::project::solution const> solutionConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::project::solution_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::project::solution_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace project

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'project': ['/home/cc/ee106a/fa16/class/ee106a-adi/ros_workspace/project/src/project/msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::project::solution_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::project::solution_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::project::solution_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::project::solution_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::project::solution_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::project::solution_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::project::solution_<ContainerAllocator> >
{
  static const char* value()
  {
    return "0eb1e6722a11b1e46249ddcba193853f";
  }

  static const char* value(const ::project::solution_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x0eb1e6722a11b1e4ULL;
  static const uint64_t static_value2 = 0x6249ddcba193853fULL;
};

template<class ContainerAllocator>
struct DataType< ::project::solution_<ContainerAllocator> >
{
  static const char* value()
  {
    return "project/solution";
  }

  static const char* value(const ::project::solution_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::project::solution_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int16 number\n\
string face\n\
float32 radian\n\
";
  }

  static const char* value(const ::project::solution_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::project::solution_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.number);
      stream.next(m.face);
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
struct Printer< ::project::solution_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::project::solution_<ContainerAllocator>& v)
  {
    s << indent << "number: ";
    Printer<int16_t>::stream(s, indent + "  ", v.number);
    s << indent << "face: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.face);
    s << indent << "radian: ";
    Printer<float>::stream(s, indent + "  ", v.radian);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PROJECT_MESSAGE_SOLUTION_H