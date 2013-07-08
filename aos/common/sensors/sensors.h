#ifndef AOS_COMMON_SENSORS_SENSORS_H_
#define AOS_COMMON_SENSORS_SENSORS_H_

#include "aos/common/time.h"
#include "aos/common/byteorder.h"
#include "aos/common/control_loop/ControlLoop.h"
#include "aos/common/inttypes.h"

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
using ::aos::control_loops::NextLoopTime;

uint32_t CalculateChecksum(char *buf, size_t size);

// This is the struct that actually gets sent over the UDP socket.
template<class Values>
struct SensorData {
  // All of the other 4-byte chunks in the message bitwise-exclusive-ORed
  // together. Needed because it seems like nobody else checks... (vxworks not
  // sending the UDP checksum or (not very likely) linux not checking it).
  // TODO(brians): static_assert that this is at the front
  uint32_t checksum;

  Values values;
  // Starts at 0 and goes up.
  int32_t count;

  void NetworkToHost() {
    count = ntoh(count);
  }
  void HostToNetwork() {
    count = hton(count);
  }

  void FillinChecksum() {
    checksum = CalculateChecksum(reinterpret_cast<char *>(this) +
                                 sizeof(checksum),
                                 sizeof(*this) - sizeof(checksum));
  }
  // Returns whether or not checksum is correct.
  bool CheckChecksum() {
    uint32_t expected = CalculateChecksum(reinterpret_cast<char *>(this) +
                                          sizeof(checksum),
                                          sizeof(*this) - sizeof(checksum));
    if (checksum != expected) {
      LOG(INFO, "expected %" PRIx32 " but got %" PRIx32 "\n",
          expected, checksum);
      return false;
    }
    return true;
  }
} __attribute__((packed));

}  // namespace sensors
}  // namespace aos

#endif  // AOS_COMMON_SENSORS_SENSORS_H_
