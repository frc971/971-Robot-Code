#include "bbb_cape/src/bbb/led.h"

#include "aos/common/logging/logging.h"

#include <string.h>
#include <errno.h>

#define DIRECTORY "/sys/class/leds/beaglebone:green:usr%d/"

namespace bbb {

LED::LED(int number) : number_(number) {
  char trigger_path[64];
  snprintf(trigger_path, sizeof(trigger_path), DIRECTORY "trigger", number_);
  FILE *trigger_handle = fopen(trigger_path, "w");
  if (trigger_handle == nullptr) {
    LOG(FATAL, "couldn't open trigger file for LED %d because of %d: %s\n",
        number_, errno, strerror(errno));
  }
  if (fputs("none", trigger_handle) < 0) {
    LOG(FATAL,
        "writing 'none' to file %p (trigger for LED %d) failed with %d: %s\n",
        trigger_handle, number_, errno, strerror(errno));
  }
  if (fclose(trigger_handle) == -1) {
    LOG(WARNING, "fclose(%p) failed with %d: %s\n",
        trigger_handle, errno, strerror(errno));
  }

  char brightness_path[64];
  snprintf(brightness_path, sizeof(brightness_path),
           DIRECTORY "brightness", number_);

  brightness_handle_ = fopen(brightness_path, "w");
  if (brightness_handle_ == nullptr) {
    LOG(FATAL, "fopen('%s', 'w') failed with %d: %s\n",
        brightness_path, errno, strerror(errno));
  }
}

LED::~LED() {
  if (fclose(brightness_handle_) == -1) {
    LOG(WARNING, "fclose(%p) failed with %d: %s\n",
        brightness_handle_, errno, strerror(errno));
  }
}

void LED::Set(bool on) {
  rewind(brightness_handle_);
  if (fputs(on ? "255" : "0", brightness_handle_) == EOF) {
    LOG(FATAL, "fputs(255|0, %p) for LED %d failed with %d: %s\n",
        brightness_handle_, number_, errno, strerror(errno));
  }
  if (fflush(brightness_handle_) == EOF) {
    LOG(FATAL, "fflush(%p) for LED %d failed with %d: %s\n",
        brightness_handle_, number_, errno, strerror(errno));
  }
}

}  // namespace bbb
