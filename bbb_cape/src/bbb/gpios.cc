#include "bbb/gpios.h"

#include <stdio.h>
#include <stdlib.h>

#include "aos/common/logging/logging_impl.h"

namespace bbb {

Pin::Pin() {}

Pin::~Pin() {
  // Unexport the pin.
  if ((handle_ = fopen("/sys/class/gpio/unexport", "ab")) == NULL) {
    LOG(WARNING, "Could not open file to unexport pin.\n");
    // There's nothing intelligent we can really do here.
    return;
  }

  fprintf(handle_, "%d", kernel_pin_);
  fclose(handle_);
}

bool Pin::InitPin(int bank, int pin) {
  kernel_pin_ = bank * 32 + pin;

  // Export the pin.
  if ((handle_ = fopen("/sys/class/gpio/export", "ab")) == NULL) {
    LOG(ERROR, "Could not open file for exporting pin.\n");
    return false;
  }

  fprintf(handle_, "%d", kernel_pin_);
  fclose(handle_);
  return true;
}

bool Pin::DoPinDirSet(int direction) {
  char type_path[64];
  snprintf(type_path, sizeof(type_path), "/sys/class/gpio/gpio%d/direction",
           kernel_pin_);

  if ((handle_ = fopen(type_path, "rb+")) == NULL) {
    LOG(ERROR, "Unable open file for pin direction setting.\n");
    return false;
  }

  switch (direction) {
    case 1:
      // Input.
      fprintf(handle_, "in");
      break;
    case 2:
      // Output.
      // If it's an output, we should specify an initial direction.
      fprintf(handle_, "low");
      break;
    default:
      LOG(ERROR, "Invalid direction identifier %d.\n", direction);
      fclose(handle_);
      return false;
  }

  fclose(handle_);
  return true;
}

}  // namespace bbb
