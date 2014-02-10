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

#include "aos/linux_code/logging/linux_logging.h"
#include "aos/linux_code/logging/binary_log_file.h"
#include "aos/linux_code/init.h"
#include "aos/linux_code/ipc_lib/queue.h"
#include "aos/common/logging/logging_impl.h"
#include "aos/common/time.h"

namespace aos {
namespace logging {
namespace linux_code {
namespace {

int LogStreamerMain() {
  InitNRT();

  const time::Time now = time::Time::Now();
  printf("starting at %" PRId32 "s%" PRId32 "ns-----------------------------\n",
         now.sec(), now.nsec());

  int index = 0;
  while (true) {
    const LogMessage *const msg = ReadNext(RawQueue::kBlock, &index);
    if (msg == NULL) continue;

    internal::PrintMessage(stdout, *msg);

    logging::linux_code::Free(msg);
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
