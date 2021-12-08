#include "aos/ipc_lib/shm_observers.h"

namespace aos {
namespace linux_code {
namespace ipc_lib {

ShmAccessorObserver before_observer = nullptr, after_observer = nullptr;

void SetShmAccessorObservers(ShmAccessorObserver before,
                             ShmAccessorObserver after) {
  before_observer = before;
  after_observer = after;
}

}  // namespace ipc_lib
}  // namespace linux_code
}  // namespace aos
