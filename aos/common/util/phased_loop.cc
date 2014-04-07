#include "aos/common/util/phased_loop.h"

#include <string.h>

#include "aos/common/logging/logging.h"
#include "aos/common/time.h"

namespace aos {
namespace time {

void PhasedLoopXMS(int ms, int offset) {
  // TODO(brians): Tests!
  const Time frequency = Time::InMS(ms);
  SleepUntil((Time::Now() / static_cast<int32_t>(frequency.ToNSec())) *
             static_cast<int32_t>(frequency.ToNSec()) +
             frequency + Time::InUS(offset));
}

}  // namespace timing
}  // namespace aos
