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

#include "aos/atom_code/logging/atom_logging.h"
#include "aos/atom_code/core/LogFileCommon.h"
#include "aos/atom_code/init.h"
#include "aos/atom_code/ipc_lib/queue.h"
#include "aos/common/logging/logging_impl.h"
#include "aos/common/time.h"

namespace aos {
namespace logging {
namespace atom {
namespace {

int LogStreamerMain() {
  InitNRT();

  const time::Time now = time::Time::Now();
  printf("starting at %" PRId32 "s%" PRId32 "ns-----------------------------\n",
         now.sec(), now.nsec());

  int index = 0;
  while (true) {
    const LogMessage *const msg = ReadNext(BLOCK, &index);
    if (msg == NULL) continue;

    internal::PrintMessage(stdout, *msg);

    logging::atom::Free(msg);
  }

  Cleanup();
  return 0;
}

}  // namespace
}  // namespace atom
}  // namespace logging
}  // namespace aos

int main() {
  return ::aos::logging::atom::LogStreamerMain();
}
