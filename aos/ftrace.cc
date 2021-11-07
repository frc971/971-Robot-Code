#include "aos/ftrace.h"

#include <cstdarg>
#include <cstdio>

DEFINE_bool(
    enable_ftrace, false,
    "If false, disable logging to /sys/kernel/debug/tracing/trace_marker");

namespace aos {

Ftrace::Ftrace()
    : message_fd_(FLAGS_enable_ftrace
                      ? open("/sys/kernel/debug/tracing/trace_marker", O_WRONLY)
                      : -1),
      on_fd_(FLAGS_enable_ftrace
                 ? open("/sys/kernel/debug/tracing/tracing_on", O_WRONLY)
                 : -1) {}

Ftrace::~Ftrace() {
  if (message_fd_ != -1) {
    PCHECK(close(message_fd_) == 0);
  }
  if (message_fd_ != -1) {
    PCHECK(close(on_fd_) == 0);
  }
}

void Ftrace::FormatMessage(const char *format, ...) {
  if (message_fd_ == -1) {
    return;
  }
  char buffer[512];
  va_list ap;
  va_start(ap, format);
  const int result = vsnprintf(buffer, sizeof(buffer), format, ap);
  va_end(ap);
  CHECK_LE(static_cast<size_t>(result), sizeof(buffer))
      << ": Format string ended up too long: " << format;
  WriteMessage(std::string_view(buffer, result));
}

void Ftrace::WriteMessage(std::string_view content) {
  if (message_fd_ == -1) {
    return;
  }
  const int result = write(message_fd_, content.data(), content.size());
  if (result == -1 && errno == EBADF) {
    // This just means tracing is turned off. Ignore it.
    return;
  }
  PCHECK(result >= 0) << ": Failed to write ftrace message: " << content;
  CHECK_EQ(static_cast<size_t>(result), content.size())
      << ": Failed to write complete ftrace message: " << content;
}

}  // namespace aos
