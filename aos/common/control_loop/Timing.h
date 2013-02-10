#ifndef __AOS_TIMEOUT_H_
#define __AOS_TIMEOUT_H_

#include <time.h>
#include <string>

namespace aos {
namespace time {

// Will not be accurate if ms isn't a factor of 1000.
// offset is in us.
void PhasedLoopXMS(int ms, int offset);
// offset is in us.
inline void PhasedLoop10MS(int offset) { PhasedLoopXMS(10, offset); }

}  // namespace time
}  // namespace aos

#endif
