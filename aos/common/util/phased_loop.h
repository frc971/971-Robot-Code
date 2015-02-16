#ifndef AOS_COMMON_UTIL_PHASED_LOOP_H_
#define AOS_COMMON_UTIL_PHASED_LOOP_H_

#include <time.h>
#include <string>

namespace aos {
namespace time {

// Will not be accurate if ms isn't a factor of 1000.
// offset is in us.
void PhasedLoopXMS(int ms, int offset);

}  // namespace time
}  // namespace aos

#endif  // AOS_COMMON_UTIL_PHASED_LOOP_H_
