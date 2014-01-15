#ifndef AOS_LINUX_CODE_CONFIGURATION_H_
#define AOS_LINUX_CODE_CONFIGURATION_H_

#include <stdint.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

namespace aos {

// Holds global configuration data. All of the functions are safe to call
// from wherever.
namespace configuration {

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

#endif  // AOS_LINUX_CODE_CONFIGURATION_H_
