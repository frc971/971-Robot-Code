#ifndef AOS_COMMON_SENSORS_SENSORS_H_
#define AOS_COMMON_SENSORS_SENSORS_H_

#include "aos/common/time.h"
#include "aos/common/byteorder.h"

#include "aos/common/control_loop/ControlLoop.h"

namespace aos {
// This namespace contains all of the stuff for dealing with reading sensors and
// communicating it to everything that needs it. There are 4 main classes whose
// instances actually process the data. They must all be registered in the
// appropriate ::aos::crio::ControlsManager hooks.
//
// SensorPackers get run on the cRIO to read inputs (from WPILib or elsewhere)
// and put the values into the Values struct (which is templated for all of the
// classes that use it).
// SensorUnpackers get run on both the atom and the cRIO to take the data from
// the Values struct and put them into queues for control loops etc.
// SensorBroadcasters (on the cRIO) send the data to a SensorReceiver (on the
// atom) to pass to its SensorUnpacker there.
// CRIOControlLoopRunners register with a SensorBroadcaster to get called right
// after reading the sensor data so that they can immediately pass it so a
// SensorUnpacker and then run their control loops.
// The actual SensorPacker and SensorUnpacker classes have the Interface suffix
// on them.
namespace sensors {

// How many times per ::aos::control_loops::kLoopFrequency sensor
// values get sent out by the cRIO.
// This must evenly divide that frequency into multiples of sysClockRateGet().
const int kSendsPerCycle = 10;
// ::aos::control_loops::kLoopFrequency / kSendsPerCycle for
// convenience.
extern const time::Time kSensorSendFrequency;
using ::aos::control_loops::kLoopFrequency;

// This is the struct that actually gets sent over the UDP socket.
template<class Values>
struct SensorData {
  Values values;
  // Starts at 0 and goes up.
  int32_t count;

  void NetworkToHost() {
    count = ntoh(count);
  }
} __attribute__((packed));

}  // namespace sensors
}  // namespace aos

#endif  // AOS_COMMON_SENSORS_SENSORS_H_
