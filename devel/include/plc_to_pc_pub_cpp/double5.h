// Generated by gencpp from file plc_to_pc_pub_cpp/double5.msg
// DO NOT EDIT!


#ifndef PLC_TO_PC_PUB_CPP_MESSAGE_DOUBLE5_H
#define PLC_TO_PC_PUB_CPP_MESSAGE_DOUBLE5_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace plc_to_pc_pub_cpp
{
template <class ContainerAllocator>
struct double5_
{
  typedef double5_<ContainerAllocator> Type;

  double5_()
    : alpha()  {
      alpha.assign(0.0);
  }
  double5_(const ContainerAllocator& _alloc)
    : alpha()  {
  (void)_alloc;
      alpha.assign(0.0);
  }



   typedef boost::array<double, 5>  _alpha_type;
  _alpha_type alpha;





  typedef boost::shared_ptr< ::plc_to_pc_pub_cpp::double5_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::plc_to_pc_pub_cpp::double5_<ContainerAllocator> const> ConstPtr;

}; // struct double5_

typedef ::plc_to_pc_pub_cpp::double5_<std::allocator<void> > double5;

typedef boost::shared_ptr< ::plc_to_pc_pub_cpp::double5 > double5Ptr;
typedef boost::shared_ptr< ::plc_to_pc_pub_cpp::double5 const> double5ConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::plc_to_pc_pub_cpp::double5_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::plc_to_pc_pub_cpp::double5_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::plc_to_pc_pub_cpp::double5_<ContainerAllocator1> & lhs, const ::plc_to_pc_pub_cpp::double5_<ContainerAllocator2> & rhs)
{
  return lhs.alpha == rhs.alpha;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::plc_to_pc_pub_cpp::double5_<ContainerAllocator1> & lhs, const ::plc_to_pc_pub_cpp::double5_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace plc_to_pc_pub_cpp

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::plc_to_pc_pub_cpp::double5_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::plc_to_pc_pub_cpp::double5_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::plc_to_pc_pub_cpp::double5_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::plc_to_pc_pub_cpp::double5_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::plc_to_pc_pub_cpp::double5_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::plc_to_pc_pub_cpp::double5_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::plc_to_pc_pub_cpp::double5_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dc8dffcd3a49516a92785ed44605ab1e";
  }

  static const char* value(const ::plc_to_pc_pub_cpp::double5_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xdc8dffcd3a49516aULL;
  static const uint64_t static_value2 = 0x92785ed44605ab1eULL;
};

template<class ContainerAllocator>
struct DataType< ::plc_to_pc_pub_cpp::double5_<ContainerAllocator> >
{
  static const char* value()
  {
    return "plc_to_pc_pub_cpp/double5";
  }

  static const char* value(const ::plc_to_pc_pub_cpp::double5_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::plc_to_pc_pub_cpp::double5_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64[5] alpha\n"
"\n"
;
  }

  static const char* value(const ::plc_to_pc_pub_cpp::double5_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::plc_to_pc_pub_cpp::double5_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.alpha);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct double5_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::plc_to_pc_pub_cpp::double5_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::plc_to_pc_pub_cpp::double5_<ContainerAllocator>& v)
  {
    s << indent << "alpha[]" << std::endl;
    for (size_t i = 0; i < v.alpha.size(); ++i)
    {
      s << indent << "  alpha[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.alpha[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // PLC_TO_PC_PUB_CPP_MESSAGE_DOUBLE5_H
