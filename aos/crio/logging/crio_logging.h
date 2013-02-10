#ifndef AOS_CRIO_CRIO_LOGGING_LOGGING_H_
#define AOS_CRIO_CRIO_LOGGING_LOGGING_H_

#ifndef AOS_COMMON_LOGGING_LOGGING_H_
#error This file may only be #included through common/logging/logging.h!!!
#endif

#undef extern
#undef const
#undef ERROR

#include <msgQLib.h>
#include <stdint.h>

//#define LOG(level, fmt, args...) printf(STRINGIFY(level) ": " fmt, ##args)
#define LOG(level, fmt, args...) \
    ::aos::logging::Do(level, \
                       LOG_SOURCENAME ": " STRINGIFY(__LINE__) ": " fmt, \
                       ##args)
//#define LOG(...)

namespace aos {
namespace logging {

// Initialize internal variables and start up the task that sends the logs to
// the atom. Must be called before Do.
void Start();
// The function that the LOG macro actually calls. Queues up a message for the
// task to send. Start must be called before this function is.
int Do(log_level level, const char *format, ...)
    __attribute__((format(printf, 2, 3)));

}  // namespace logging
}  // namespace aos

#endif  // AOS_CRIO_CRIO_LOGGING_LOGGING_H_
