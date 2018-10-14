#include "aos/libc/aos_strerror.h"

#include <assert.h>
#include <sys/types.h>
#include <string.h>
#include <stdio.h>

// This code uses an overloaded function to handle the result from either
// version of strerror_r correctly without needing a way to get the choice out
// of the compiler/glibc/whatever explicitly.

namespace {

const size_t kBufferSize = 128;

// Handle the result from the GNU version of strerror_r. It never fails, so
// that's pretty easy...
__attribute__((unused))
char *aos_strerror_handle_result(int /*error*/, char *ret, char * /*buffer*/) {
  return ret;
}

// Handle the result from the POSIX version of strerror_r.
__attribute__((unused))
char *aos_strerror_handle_result(int error, int ret, char *buffer) {
  if (ret != 0) {
#ifndef NDEBUG
    // assert doesn't use the return value when building optimized.
    const int r =
#endif
        snprintf(buffer, kBufferSize, "Unknown error %d", error);
    assert(r > 0);
  }
  return buffer;
}

}  // namespace

const char *aos_strerror(int error) {
  static thread_local char buffer[kBufferSize];

  // Call the overload for whichever version we're using.
  return aos_strerror_handle_result(
      error, strerror_r(error, buffer, sizeof(buffer)), buffer);
}
