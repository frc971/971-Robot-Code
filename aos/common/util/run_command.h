#ifndef AOS_COMMON_UTIL_RUN_COMMAND_H_
#define AOS_COMMON_UTIL_RUN_COMMAND_H_

namespace aos {
namespace util {

// Improved replacement for system(3). Doesn't block signals like system(3) and
// is thread-safe. Also makes sure all 3 standard streams are /dev/null.
//
// This means that it passes command to `/bin/sh -c` and returns -1 or a status
// like from wait(2).
int RunCommand(const char *command);

}  // namespace util
}  // namespace aos

#endif  // AOS_COMMON_UTIL_RUN_COMMAND_H_
