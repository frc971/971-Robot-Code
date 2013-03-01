#ifndef AOS_COMMON_ZERO_SWITCH_VALUE_H_
#define AOS_COMMON_ZERO_SWITCH_VALUE_H_

#include "aos/common/type_traits.h"
#include "aos/common/byteorder.h"

namespace aos {

// Contains all of the information from a zeroing sensor of some kind.
// It is all contained here because it all has to be retrieved at the same time
// or else there are race conditions with the sensor triggering and retrieving
// the encoder values that would make writing code to deal with the information
// hard (ie if the trigger sensor is read, then it changes->triggers the
// interrupt which reads the encoder value).
struct ZeroSwitchValue {
  // What the curent encoder value is.
  int32_t current_encoder;
  // What the value of the encoder was when the interrupt triggered.
  int32_t edge_encoder;
  // What the current value of the sensor is.
  bool current_value;

  void NetworkToHost() {
    current_encoder = ntoh(current_encoder);
    edge_encoder = ntoh(edge_encoder);
  }
};
static_assert(shm_ok<ZeroSwitchValue>::value, "it's getting sent over the wire");

}  // namespace aos

#endif  // AOS_COMMON_ZERO_SWITCH_VALUE_H_
