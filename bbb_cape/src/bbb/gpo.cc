#include "bbb/gpo.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "aos/common/logging/logging.h"

namespace bbb {

Gpo::Gpo(int bank, int pin, bool initial_value)
    : GpioPin(bank, pin, false, initial_value) {}

void Gpo::Set(bool high) {
  rewind(value_handle_);
  if (fputc(high ? '1' : '0', value_handle_) == EOF) {
    PLOG(FATAL, "fputc(%c, %p) for pin (%d,%d) failed",
         high ? '1': '0', value_handle_, bank_, pin_);
  }
  if (fflush(value_handle_) == EOF) {
    PLOG(FATAL, "fflush(%p) for pin (%d,%d) failed",
         value_handle_, bank_, pin_);
  }
}

}  // namespace bbb
