// Generated by gencpp from file beckhoff_msgs/array6.msg
// DO NOT EDIT!


#ifndef BECKHOFF_MSGS_MESSAGE_ARRAY6_H
#define BECKHOFF_MSGS_MESSAGE_ARRAY6_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace beckhoff_msgs
{
template <class ContainerAllocator>
struct array6_
{
  typedef array6_<ContainerAllocator> Type;

  array6_()
    : data()  {
      data.assign(0.0);
  }
  array6_(const ContainerAllocator& _alloc)
    : data()  {
  (void)_alloc;
      data.assign(0.0);
  }



   typedef boost::array<double, 6>  _data_type;
  _data_type data;





  typedef boost::shared_ptr< ::beckhoff_msgs::array6_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::beckhoff_msgs::array6_<ContainerAllocator> const> ConstPtr;

}; // struct array6_

typedef ::beckhoff_msgs::array6_<std::allocator<void> > array6;

typedef boost::shared_ptr< ::beckhoff_msgs::array6 > array6Ptr;
typedef boost::shared_ptr< ::beckhoff_msgs::array6 const> array6ConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::beckhoff_msgs::array6_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::beckhoff_msgs::array6_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::beckhoff_msgs::array6_<ContainerAllocator1> & lhs, const ::beckhoff_msgs::array6_<ContainerAllocator2> & rhs)
{
  return lhs.data == rhs.data;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::beckhoff_msgs::array6_<ContainerAllocator1> & lhs, const ::beckhoff_msgs::array6_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace beckhoff_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::beckhoff_msgs::array6_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::beckhoff_msgs::array6_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::beckhoff_msgs::array6_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::beckhoff_msgs::array6_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::beckhoff_msgs::array6_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::beckhoff_msgs::array6_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::beckhoff_msgs::array6_<ContainerAllocator> >
{
  static const char* value()
  {
    return "402f543b13bcc71e911bc511f3982ac2";
  }

  static const char* value(const ::beckhoff_msgs::array6_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x402f543b13bcc71eULL;
  static const uint64_t static_value2 = 0x911bc511f3982ac2ULL;
};

template<class ContainerAllocator>
struct DataType< ::beckhoff_msgs::array6_<ContainerAllocator> >
{
  static const char* value()
  {
    return "beckhoff_msgs/array6";
  }

  static const char* value(const ::beckhoff_msgs::array6_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::beckhoff_msgs::array6_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64[6] data\n"
;
  }

  static const char* value(const ::beckhoff_msgs::array6_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::beckhoff_msgs::array6_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.data);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct array6_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::beckhoff_msgs::array6_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::beckhoff_msgs::array6_<ContainerAllocator>& v)
  {
    s << indent << "data[]" << std::endl;
    for (size_t i = 0; i < v.data.size(); ++i)
    {
      s << indent << "  data[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.data[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // BECKHOFF_MSGS_MESSAGE_ARRAY6_H