#include "aos/crio/shared_libs/interrupt_bridge.h"

namespace aos {
namespace crio {

void **const timer_notifiers = new void *[SIGRTMAX - SIGRTMIN];

}  // namespace crio
}  // namespace aos
