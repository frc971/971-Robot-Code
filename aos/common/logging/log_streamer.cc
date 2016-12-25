#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <pwd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>
#include <chrono>
#include <string>

#include "aos/common/logging/implementations.h"
#include "aos/common/logging/logging.h"
#include "aos/common/time.h"
#include "aos/linux_code/init.h"
#include "aos/linux_code/ipc_lib/queue.h"

namespace aos {
namespace logging {
namespace linux_code {
namespace {

namespace chrono = ::std::chrono;

int LogStreamerMain() {
  InitNRT();

  RawQueue *queue = GetLoggingQueue();

  const monotonic_clock::time_point now = monotonic_clock::now();
  chrono::seconds sec =
      chrono::duration_cast<chrono::seconds>(now.time_since_epoch());
  chrono::nanoseconds nsec =
      chrono::duration_cast<chrono::nanoseconds>(now.time_since_epoch() - sec);
  printf("starting at %" PRId32 "s%" PRId32 "ns-----------------------------\n",
         static_cast<int32_t>(sec.count()), static_cast<int32_t>(nsec.count()));

  while (true) {
    const LogMessage *const msg = static_cast<const LogMessage *>(
        queue->ReadMessage(RawQueue::kNonBlock));
    if (msg == NULL) {
      ::std::this_thread::sleep_for(::std::chrono::milliseconds(100));
    } else {
      internal::PrintMessage(stdout, *msg);

      queue->FreeMessage(msg);
    }
  }

  Cleanup();
  return 0;
}

}  // namespace
}  // namespace linux_code
}  // namespace logging
}  // namespace aos

int main() {
  return ::aos::logging::linux_code::LogStreamerMain();
}
