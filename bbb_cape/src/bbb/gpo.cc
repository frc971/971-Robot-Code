#include "gpo.h"

#include <stdio.h>

#include "aos/common/logging/logging.h"

namespace bbb {

Gpo::Gpo(int bank, int pin) {
  // All the failures here are fatal, seeing as
  // if we can't initialize the pin, we're probably
  // screwed anyway.
  if (!InitPin(bank, pin)) {
    LOG(FATAL, "Failed to export the pin.\n");
  }
  if (!DoPinDirSet(2)) {
    LOG(FATAL, "Failed to make the pin an output.\n");
  }
}

bool Gpo::Set(const bool high) {
  char val_path[64];
  snprintf(val_path, sizeof(val_path), "/sys/class/gpio/gpio%d/value",
           kernel_pin_);

  if ((handle_ = fopen(val_path, "rb+")) == NULL) {
    LOG(ERROR, "Unable open file for pin value setting.\n");
    return false;
  }

  fprintf(handle_, "%d", high);
  fclose(handle_);

  return true;
}

}  // namespace bbb
