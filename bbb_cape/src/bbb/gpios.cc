#include "bbb/gpios.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "aos/common/logging/logging.h"

namespace bbb {

GpioPin::GpioPin(int bank, int pin, bool input, bool initial_value)
    : bank_(bank), pin_(pin), kernel_pin_(bank * 32 + pin) {
  // Export the pin.
  FILE *export_handle = fopen("/sys/class/gpio/export", "w");
  if (export_handle == NULL) {
    PLOG(WARNING, "Could not open file to export pin (%d,%d)", bank_, pin_);
  } else {
    if (fprintf(export_handle, "%d", kernel_pin_) < 0) {
      PLOG(WARNING, "Could not write to file %p to export pin (%d,%d)",
           export_handle, bank_, pin_);
    }
    if (fclose(export_handle) == -1) {
      PLOG(WARNING, "fclose(%p) failed", export_handle);
    }
  }

  char direction_path[64];
  snprintf(direction_path, sizeof(direction_path),
           "/sys/class/gpio/gpio%d/direction", kernel_pin_);

  FILE *direction_handle = fopen(direction_path, "w");
  if (direction_handle == NULL) {
    PLOG(FATAL, "fopen(%s, \"w\") failed", direction_path);
  }

  if (fputs(input ? "in" : (initial_value ? "high" : "low"),
            direction_handle) < 0) {
    PLOG(FATAL, "setting direction for pin (%d,%d) failed", bank_, pin_);
  }
  if (fclose(direction_handle) == -1) {
    PLOG(WARNING, "fclose(%p) failed", direction_handle);
  }

  char value_path[64];
  snprintf(value_path, sizeof(value_path),
           "/sys/class/gpio/gpio%d/value", kernel_pin_);
  value_handle_ = fopen(value_path, "w+");
  if (value_handle_ == NULL) {
    PLOG(FATAL, "fopen(%s, \"w+\") failed", value_path);
  }
}

GpioPin::~GpioPin() {
  // Unexport the pin.
  FILE *unexport_handle = fopen("/sys/class/gpio/unexport", "a");
  if (unexport_handle == NULL) {
    PLOG(WARNING, "Could not open file to unexport pin (%d,%d)", bank_, pin_);
  } else {
    if (fprintf(unexport_handle, "%d", kernel_pin_) < 0) {
      PLOG(WARNING, "Could not write to file %p to unexport pin (%d,%d)",
           unexport_handle, bank_, pin_);
    }
    if (fclose(unexport_handle) == -1) {
      PLOG(WARNING, "fclose(%p) failed", unexport_handle);
    }
  }
}

}  // namespace bbb
