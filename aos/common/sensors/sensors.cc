#include "aos/common/sensors/sensors.h"

namespace aos {
namespace sensors {

const time::Time kSensorSendFrequency =
    ::aos::control_loops::kLoopFrequency / kSendsPerCycle;

}  // namespace sensors
}  // namespace aos
