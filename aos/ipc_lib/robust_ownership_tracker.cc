#include "aos/ipc_lib/robust_ownership_tracker.h"

#include "aos/ipc_lib/lockless_queue.h"

namespace aos {
namespace ipc_lib {

bool RobustOwnershipTracker::PretendThatOwnerIsDeadForTesting(pid_t died_tid) {
  return ipc_lib::PretendThatOwnerIsDeadForTesting(&mutex_, died_tid);
}

::std::string RobustOwnershipTracker::DebugString() const {
  ::std::stringstream s;
  s << "{.tid=aos_mutex(" << ::std::hex << mutex_.futex;

  if (mutex_.futex != 0) {
    s << ":";
    if (mutex_.futex & FUTEX_OWNER_DIED) {
      s << "FUTEX_OWNER_DIED|";
    }
    s << "tid=" << (mutex_.futex & FUTEX_TID_MASK);
  }

  s << "),}";
  return s.str();
}

}  // namespace ipc_lib
}  // namespace aos
