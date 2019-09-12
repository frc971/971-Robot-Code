#include "aos/ipc_lib/index.h"

#include <string>
#include <sstream>

namespace aos {
namespace ipc_lib {

::std::string QueueIndex::DebugString() const {
  if (valid()) {
    ::std::stringstream s;
    s << "QueueIndex(" << index_ << "/0x" << ::std::hex << index_ << ::std::dec
      << ", count=" << count_ << ")";
    return s.str();
  } else {
    return "QueueIndex::Invalid()";
  }
}

::std::string Index::DebugString() const {
  if (valid()) {
    ::std::stringstream s;
    s << "Index(queue_index=" << queue_index() << "/0x" << ::std::hex
      << queue_index() << ::std::dec << ", message_index=" << message_index()
      << ")";
    return s.str();
  } else {
    return "QueueIndex::Invalid()";
  }
}

}  // namespace ipc_lib
}  // namespace aos
