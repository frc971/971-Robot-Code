#include "bbb/gpi.h"

#include <stdio.h>
#include <errno.h>
#include <string.h>

#include "aos/common/logging/logging.h"

namespace bbb {

Gpi::Gpi(int bank, int pin) : GpioPin(bank, pin, true) {
}

bool Gpi::Read() {
  rewind(value_handle_);
  int value = fgetc(value_handle_);
  if (value < 0) {
    LOG(FATAL, "fgetc(%p) for pin (%d,%d) failed with %d: %s\n",
        value_handle_, bank_, pin_, errno, strerror(errno));
  }
  switch (value - '0') {
    case 0: return false;
    case 1: return true;
    default:
      LOG(FATAL, "unknown pin value %c\n", value);
  }
}

}  // namespace bbb
