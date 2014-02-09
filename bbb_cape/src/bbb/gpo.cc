#include "bbb/gpo.h"

#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>

#include "aos/common/logging/logging.h"

namespace bbb {

Gpo::Gpo(int bank, int pin, bool initial_value)
    : GpioPin(bank, pin, false, initial_value) {}

void Gpo::Set(bool high) {
  // TODO(brians): Figure out why this breaks it.
  //rewind(value_handle_);
  if (fputc(high ? '1' : '0', value_handle_) == EOF) {
    LOG(FATAL, "fputc(%c, %p) for pin (%d,%d) failed with %d: %s\n",
        high ? '1': '0', value_handle_, bank_, pin_, errno, strerror(errno));
  }
  sync();
}

}  // namespace bbb
