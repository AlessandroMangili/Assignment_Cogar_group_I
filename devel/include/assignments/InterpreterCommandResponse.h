// Generated by gencpp from file assignments/InterpreterCommandResponse.msg
// DO NOT EDIT!


#ifndef ASSIGNMENTS_MESSAGE_INTERPRETERCOMMANDRESPONSE_H
#define ASSIGNMENTS_MESSAGE_INTERPRETERCOMMANDRESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace assignments
{
template <class ContainerAllocator>
struct InterpreterCommandResponse_
{
  typedef InterpreterCommandResponse_<ContainerAllocator> Type;

  InterpreterCommandResponse_()
    : valid(false)
    , interpreted_action()  {
    }
  InterpreterCommandResponse_(const ContainerAllocator& _alloc)
    : valid(false)
    , interpreted_action(_alloc)  {
  (void)_alloc;
    }



   typedef uint8_t _valid_type;
  _valid_type valid;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _interpreted_action_type;
  _interpreted_action_type interpreted_action;





  typedef boost::shared_ptr< ::assignments::InterpreterCommandResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::assignments::InterpreterCommandResponse_<ContainerAllocator> const> ConstPtr;

}; // struct InterpreterCommandResponse_

typedef ::assignments::InterpreterCommandResponse_<std::allocator<void> > InterpreterCommandResponse;

typedef boost::shared_ptr< ::assignments::InterpreterCommandResponse > InterpreterCommandResponsePtr;
typedef boost::shared_ptr< ::assignments::InterpreterCommandResponse const> InterpreterCommandResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::assignments::InterpreterCommandResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::assignments::InterpreterCommandResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::assignments::InterpreterCommandResponse_<ContainerAllocator1> & lhs, const ::assignments::InterpreterCommandResponse_<ContainerAllocator2> & rhs)
{
  return lhs.valid == rhs.valid &&
    lhs.interpreted_action == rhs.interpreted_action;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::assignments::InterpreterCommandResponse_<ContainerAllocator1> & lhs, const ::assignments::InterpreterCommandResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace assignments

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::assignments::InterpreterCommandResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::assignments::InterpreterCommandResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::assignments::InterpreterCommandResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::assignments::InterpreterCommandResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::assignments::InterpreterCommandResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::assignments::InterpreterCommandResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::assignments::InterpreterCommandResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "99879187b30a6b5efe9afd019ea80873";
  }

  static const char* value(const ::assignments::InterpreterCommandResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x99879187b30a6b5eULL;
  static const uint64_t static_value2 = 0xfe9afd019ea80873ULL;
};

template<class ContainerAllocator>
struct DataType< ::assignments::InterpreterCommandResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "assignments/InterpreterCommandResponse";
  }

  static const char* value(const ::assignments::InterpreterCommandResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::assignments::InterpreterCommandResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Response\n"
"bool valid\n"
"string interpreted_action\n"
;
  }

  static const char* value(const ::assignments::InterpreterCommandResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::assignments::InterpreterCommandResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.valid);
      stream.next(m.interpreted_action);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct InterpreterCommandResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::assignments::InterpreterCommandResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::assignments::InterpreterCommandResponse_<ContainerAllocator>& v)
  {
    s << indent << "valid: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.valid);
    s << indent << "interpreted_action: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.interpreted_action);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ASSIGNMENTS_MESSAGE_INTERPRETERCOMMANDRESPONSE_H
