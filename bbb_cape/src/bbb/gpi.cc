#include "bbb/gpi.h"

#include <stdio.h>
#include <string.h>

#include "aos/common/logging/logging.h"

namespace bbb {

Gpi::Gpi(int bank, int pin) : GpioPin(bank, pin, true) {
}

bool Gpi::Read() {
  rewind(value_handle_);
  int value = fgetc(value_handle_);
  if (value < 0) {
    PLOG(FATAL, "fgetc(%p) for pin (%d,%d) failed", value_handle_, bank_, pin_);
  }
  switch (value - '0') {
    case 0: return false;
    case 1: return true;
    default:
      LOG(FATAL, "unknown pin value %c\n", value);
  }
}

}  // namespace bbb
