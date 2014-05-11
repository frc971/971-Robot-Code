#include "bbb_cape/src/bbb/led.h"

#include "aos/common/logging/logging.h"

#include <string.h>

#define DIRECTORY "/sys/class/leds/beaglebone:green:usr%d/"

namespace bbb {

LED::LED(int number) : number_(number) {
  char trigger_path[64];
  snprintf(trigger_path, sizeof(trigger_path), DIRECTORY "trigger", number_);
  FILE *trigger_handle = fopen(trigger_path, "w");
  if (trigger_handle == nullptr) {
    PLOG(FATAL, "couldn't open trigger file for LED %d", number_);
  }
  if (fputs("none", trigger_handle) < 0) {
    PLOG(FATAL, "writing 'none' to file %p (trigger for LED %d) failed",
         trigger_handle, number_);
  }
  if (fclose(trigger_handle) == -1) {
    PLOG(WARNING, "fclose(%p) failed", trigger_handle);
  }

  char brightness_path[64];
  snprintf(brightness_path, sizeof(brightness_path),
           DIRECTORY "brightness", number_);

  brightness_handle_ = fopen(brightness_path, "w");
  if (brightness_handle_ == nullptr) {
    PLOG(FATAL, "fopen('%s', 'w') failed", brightness_path);
  }
}

LED::~LED() {
  if (fclose(brightness_handle_) == -1) {
    PLOG(WARNING, "fclose(%p) failed", brightness_handle_);
  }
}

void LED::Set(bool on) {
  rewind(brightness_handle_);
  if (fputs(on ? "255" : "0", brightness_handle_) == EOF) {
    PLOG(FATAL, "fputs(255|0, %p) for LED %d failed",
         brightness_handle_, number_);
  }
  if (fflush(brightness_handle_) == EOF) {
    PLOG(FATAL, "fflush(%p) for LED %d failed",
         brightness_handle_, number_);
  }
}

}  // namespace bbb
