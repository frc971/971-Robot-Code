#include "aos/common/logging/logging.h"
#include "aos/common/control_loop/Timing.h"

#include "aos/common/time.h"

namespace aos {
namespace time {

void PhasedLoopXMS(int ms, int offset) {
  // TODO(brians): Rewrite this cleaner.
  // TODO(brians): Tests!
  int64_t period_nsec = Time::InMS(ms).nsec();
  SleepUntil(Time::InNS((Time::Now().ToNSec() / period_nsec +
                         static_cast<int64_t>(1)) * period_nsec +
                        Time::InUS(offset).ToNSec()));
}

}  // namespace timing
}  // namespace aos
