#ifndef AOS_COMMON_NETWORK_PORT_H_
#define AOS_COMMON_NETWORK_PORT_H_

#include <stdint.h>

namespace aos {

// Constants representing the various ports used for communications and some
// documentation about what each is used for.
enum class NetworkPort : uint16_t {
  // UDP socket sending motor values from the prime to the crio.
  kMotors = 9710,
  // UDP socket forwarding drivers station packets from the crio to the prime.
  kDS = 9711,
  // UDP socket sending sensor values from the crio to the prime.
  kSensors = 9712,
  // HTTP server that sends out camera feeds in mjpg format.
  // Should not be changed because this number shows up elsewhere too.
  kCameraStreamer = 9714,
};

// Constants representing the various devices that talk on the network and the
// last segment of their IP addresses.
enum class NetworkAddress : uint8_t {
  // The computer that the cRIO talks to.
  kPrime = 179,
  kCRIO = 2,
};

}  // namespace aos

#endif  // AOS_COMMON_NETWORK_PORT_H_
