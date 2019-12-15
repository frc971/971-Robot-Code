#ifndef AOS_LOGGING_LOG_NAMER_H_
#define AOS_LOGGING_LOG_NAMER_H_

#include <string>

namespace aos {
namespace logging {
// Returns the correct filename to log to, blocking until the usb drive
// filesystem mounts, incrementing the number on the end of the filename, and
// setting up a symlink at basename-current.
// basename is the prefix to use for the logs within the usb drive. E.g., on a
// typical roborio setup, calling GetLogName("abc") will return a filename of
// the form "/media/sda1/abc-123" and setup a symlink pointing to it at
// "/media/sda1/abc-current".
std::string GetLogName(const char *basename);
}  // namespace logging
}  // namespace aos

#endif  // AOS_LOGGING_LOG_NAMER_H_
