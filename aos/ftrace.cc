#include "aos/ftrace.h"

#include <stdarg.h>
#include <stdio.h>

namespace aos {

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
