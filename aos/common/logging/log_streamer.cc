#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>
#include <string.h>
#include <string>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include <fcntl.h>
#include <inttypes.h>

#include "aos/linux_code/init.h"
#include "aos/linux_code/ipc_lib/queue.h"
#include "aos/common/logging/logging.h"
#include "aos/common/logging/implementations.h"
#include "aos/common/time.h"

namespace aos {
namespace logging {
namespace linux_code {
namespace {

int LogStreamerMain() {
  InitNRT();

  RawQueue *queue = GetLoggingQueue();

  const time::Time now = time::Time::Now();
  printf("starting at %" PRId32 "s%" PRId32 "ns-----------------------------\n",
         now.sec(), now.nsec());

  while (true) {
    const LogMessage *const msg = static_cast<const LogMessage *>(
        queue->ReadMessage(RawQueue::kNonBlock));
    if (msg == NULL) {
      ::aos::time::SleepFor(::aos::time::Time::InSeconds(0.1));
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
