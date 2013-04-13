#ifndef AOS_COMMON_CONFIGURATION_H_
#define AOS_COMMON_CONFIGURATION_H_

#include "aos/aos_stdint.h"

namespace aos {

// Constants representing the various ports used for communications and some
// documentation about what each is used for.
enum class NetworkPort : uint16_t {
  // UDP socket sending motor values from the atom to the crio.
  kMotors = 9710,
  // UDP socket forwarding drivers station packets from the crio to the atom.
  kDS = 9711,
  // UDP socket sending sensor values from the crio to the atom.
  kSensors = 9712,
  // TCP socket(s) (automatically reconnects) sending logs from the crio to the
  // atom.
  kLogs = 9713,
  // HTTP server that sends out camera feeds in mjpg format.
  // Should not be changed because this number shows up elsewhere too.
  kCameraStreamer = 9714,
};

// Holds global configuration data. All of the functions are safe to call
// from wherever (the ones that need to create locks on the cRIO).
namespace configuration {

// Constants indentifying various devices on the network.
enum class NetworkDevice {
  // The computer that the cRIO talks to.
  kAtom,
  kCRIO,
  // Whatever device this is being run on.
  kSelf,
};
// Returns the IP address to get to the specified machine.
// The return value should be passed to free(3) if it is no longer needed.
const char *GetIPAddress(NetworkDevice device);

// Returns the "root directory" for this run. Under linux, this is the
// directory where the executable is located (from /proc/self/exe) and under
// vxworks it is just "/".
// The return value will always be to a static string, so no freeing is
// necessary.
const char *GetRootDirectory();
// Returns the directory where logs get written. Relative to GetRootDirectory().
// The return value will always be to a static string, so no freeing is
// necessary.
const char *GetLoggingDirectory();

}  // namespace configuration
}  // namespace aos

#endif
