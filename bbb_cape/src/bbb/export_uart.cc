#include "bbb/export_uart.h"

#include <unistd.h>
#include <errno.h>
#include <string.h>

#include "aos/common/logging/logging.h"
#include "aos/common/time.h"

namespace bbb {
namespace {

// TODO(brians): Determine this in some way that allows easy switching for
// testing with /dev/ttyUSB0 for example.
const char *device = "/dev/ttyO1";

bool easy_access(const char *path) {
  if (access(path, R_OK | W_OK) == 0) return true;
  if (errno == EACCES || errno == ENOENT) return false;
  LOG(FATAL, "access(%s, F_OK) failed with %d: %s\n", path, errno,
      strerror(errno));
}

}  // namespace

const char *UartDevice() { return device; }

void ExportUart() {
  if (easy_access(device)) {
    LOG(INFO, "unexporting BB-UART1\n");
    if (system("bash -c 'echo -$(cat /sys/devices/bone_capemgr.*/slots"
               " | fgrep BB-UART1"
               " | cut -d : -f 1 | tr -d \" \")"
               " > /sys/devices/bone_capemgr.*/slots'") == -1) {
      LOG(FATAL, "system([disable OMAP UART]) failed with %d: %s\n", errno,
          strerror(errno));
    }
    while (easy_access(device)) {
      LOG(DEBUG, "waiting for BB-UART1 to be unexported\n");
      ::aos::time::SleepFor(::aos::time::Time::InSeconds(0.1));
    }
  }

  LOG(INFO, "exporting BB-UART1\n");
  // 2 strings to work around a VIM bug where the indenter locks up when they're
  // combined as 1...
  if (system("bash -c 'echo BB-UART1 > /sys/devices/bone_capemgr.*"
             "/slots'") ==
      -1) {
    LOG(FATAL, "system([enable OMAP UART]) failed with %d: %s\n", errno,
        strerror(errno));
  }
  while (!easy_access(device)) {
    LOG(DEBUG, "waiting for BB-UART1 to be exported\n");
    ::aos::time::SleepFor(::aos::time::Time::InSeconds(0.1));
  }
}

}  // namespace bbb
