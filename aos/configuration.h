#ifndef AOS_CONFIGURATION_H_
#define AOS_CONFIGURATION_H_

#include <stdint.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "absl/strings/string_view.h"
#include "aos/configuration_generated.h"
#include "aos/flatbuffers.h"

namespace aos {

// Holds global configuration data. All of the functions are safe to call
// from wherever.
namespace configuration {

// Reads a json configuration.  This includes all imports and merges.  Note:
// duplicate imports will result in a CHECK.
Flatbuffer<Configuration> ReadConfig(const absl::string_view path);

// Returns the resolved location for a name, type, and application name.
//
// If the application name is empty, it is ignored.  Maps are processed in
// reverse order, and application specific first.
const Location *GetLocation(const Flatbuffer<Configuration> &config,
                            const absl::string_view name,
                            const absl::string_view type,
                            const absl::string_view application_name);

// Returns "our" IP address.
const in_addr &GetOwnIPAddress();

// Returns the "root directory" for this run. Under linux, this is the
// directory where the executable is located (from /proc/self/exe)
// The return value will always be to a static string, so no freeing is
// necessary.
const char *GetRootDirectory();
// Returns the directory where logs get written. Relative to GetRootDirectory().
// The return value will always be to a static string, so no freeing is
// necessary.
const char *GetLoggingDirectory();

}  // namespace configuration
}  // namespace aos

#endif  // AOS_CONFIGURATION_H_
