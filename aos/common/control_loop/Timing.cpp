#include "aos/common/control_loop/Timing.h"

#include <string.h>

#include "aos/common/logging/logging.h"
#include "aos/common/time.h"

namespace aos {
namespace time {

void PhasedLoopXMS(int ms, int offset) {
  // TODO(brians): Tests!
  const Time frequency = Time::InMS(ms);
  SleepUntil((Time::Now() / frequency.ToNSec()) *
             frequency.ToNSec() +
             frequency + Time::InUS(offset));
}

}  // namespace timing
}  // namespace aos
