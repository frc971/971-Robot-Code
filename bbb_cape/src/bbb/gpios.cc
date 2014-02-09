#include "bbb/gpios.h"

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>

#include "aos/common/logging/logging.h"

namespace bbb {

GpioPin::GpioPin(int bank, int pin, bool input, bool initial_value)
    : bank_(bank), pin_(pin), kernel_pin_(bank * 32 + pin) {
  // Export the pin.
  FILE *export_handle = fopen("/sys/class/gpio/export", "a");
  if (export_handle == NULL) {
    LOG(WARNING,
        "Could not open file to export pin (%d,%d) because of %d: %s.\n",
        bank_, pin_, errno, strerror(errno));
  } else {
    if (fprintf(export_handle, "%d", kernel_pin_) < 0) {
      LOG(WARNING, "Could not write to file %p to export pin (%d,%d) because "
                   "of %d: %s.\n",
          export_handle, bank_, pin_, errno, strerror(errno));
    }
    if (fclose(export_handle) == -1) {
      LOG(WARNING, "fclose(%p) failed with %d: %s\n", export_handle, errno,
          strerror(errno));
    }
  }

  char direction_path[64];
  snprintf(direction_path, sizeof(direction_path),
           "/sys/class/gpio/gpio%d/direction", kernel_pin_);

  FILE *direction_handle = fopen(direction_path, "w");
  if (direction_handle == NULL) {
    LOG(FATAL, "fopen(%s, \"w+\") failed with %d: %s\n",
        direction_path, errno, strerror(errno));
  }

  if (fputs(input ? "in" : (initial_value ? "high" : "low"),
            direction_handle) < 0) {
    LOG(FATAL, "setting direction for pin (%d,%d) failed with %d: %s\n",
        bank_, pin_, errno, strerror(errno));
  }
  if (fclose(direction_handle) == -1) {
    LOG(WARNING, "fclose(%p) failed with %d: %s\n", direction_handle, errno,
        strerror(errno));
  }

  char value_path[64];
  snprintf(value_path, sizeof(value_path),
           "/sys/class/gpio/gpio%d/value", kernel_pin_);
  value_handle_ = fopen(value_path, "w+");
  if (value_handle_ == NULL) {
    LOG(FATAL, "fopen(%s, \"rw\") failed with %d: %s\n",
        value_path, errno, strerror(errno));
  }
}

GpioPin::~GpioPin() {
  // Unexport the pin.
  FILE *unexport_handle = fopen("/sys/class/gpio/unexport", "a");
  if (unexport_handle == NULL) {
    LOG(WARNING,
        "Could not open file to unexport pin (%d,%d) because of %d: %s.\n",
        bank_, pin_, errno, strerror(errno));
  } else {
    if (fprintf(unexport_handle, "%d", kernel_pin_) < 0) {
      LOG(WARNING, "Could not write to file %p to unexport pin (%d,%d) because "
                   "of %d: %s.\n",
          unexport_handle, bank_, pin_, errno, strerror(errno));
    }
    if (fclose(unexport_handle) == -1) {
      LOG(WARNING, "fclose(%p) failed with %d: %s\n", unexport_handle, errno,
          strerror(errno));
    }
  }
}

}  // namespace bbb
