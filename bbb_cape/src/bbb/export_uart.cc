#include "bbb/export_uart.h"

#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/wait.h>

#include "aos/common/logging/logging.h"
#include "aos/common/time.h"
#include "aos/common/util/run_command.h"

namespace bbb {
namespace {

// TODO(brians): Determine this in some way that allows easy switching for
// testing with /dev/ttyUSB0 for example.
const char *device = "/dev/ttyO1";

bool easy_access(const char *path) {
  if (access(path, R_OK | W_OK) == 0) return true;
  if (errno == EACCES || errno == ENOENT) return false;
  PLOG(FATAL, "access(%s, F_OK) failed", path);
}

}  // namespace

const char *UartDevice() { return device; }

void ExportUart() {
  if (easy_access(device)) {
    LOG(INFO, "unexporting BB-UART1\n");
    const int result = ::aos::util::RunCommand(
        "bash -c 'echo -$(cat /sys/devices/bone_capemgr.*/slots"
        " | fgrep BB-UART1"
        " | cut -d : -f 1 | tr -d \" \")"
        " > /sys/devices/bone_capemgr.*/slots'");
    if (result == -1) {
      PLOG(FATAL, "RunCommand([disable OMAP UART]) failed");
    } else if (!WIFEXITED(result) || WEXITSTATUS(result) != 0) {
      LOG(FATAL, "command to disable OMAP UART failed; result = %x\n", result);
    }
    while (easy_access(device)) {
      LOG(DEBUG, "waiting for BB-UART1 to be unexported\n");
      ::aos::time::SleepFor(::aos::time::Time::InSeconds(0.1));
    }
  }

  LOG(INFO, "exporting BB-UART1\n");
  // 2 strings to work around a VIM bug where the indenter locks up when they're
  // combined as 1...
  const int result = ::aos::util::RunCommand(
      "bash -c 'echo BB-UART1 > /sys/devices/bone_capemgr.*" "/slots'");
  if (result == -1) {
    PLOG(FATAL, "RunCommand([enable OMAP UART]) failed");
  } else if (!WIFEXITED(result) || WEXITSTATUS(result) != 0) {
    LOG(FATAL, "command to enable OMAP UART failed; result = %x\n", result);
  }
  while (!easy_access(device)) {
    LOG(DEBUG, "waiting for BB-UART1 to be exported\n");
    ::aos::time::SleepFor(::aos::time::Time::InSeconds(0.1));
  }
}

}  // namespace bbb
