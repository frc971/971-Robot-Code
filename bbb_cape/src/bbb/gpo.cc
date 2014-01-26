#include "bbb/gpo.h"

#include <stdio.h>
#include <errno.h>
#include <string.h>

#include "aos/common/logging/logging.h"

namespace bbb {

Gpo::Gpo(int bank, int pin, bool initial_value)
    : GpioPin(bank, pin, false, initial_value) {}

void Gpo::Set(bool high) {
  if (fputc(high ? '1' : '0', value_handle_) < 0) {
    LOG(FATAL, "fputc(%c, %p) for pin (%d,%d) failed with %d: %s\n",
        high ? '1': '0', value_handle_, bank_, pin_, errno, strerror(errno));
  }
}

}  // namespace bbb
