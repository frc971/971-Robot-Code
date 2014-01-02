#include "gpi.h"

#include <stdio.h>

#include "aos/common/logging/logging.h"

namespace bbb {

Gpi::Gpi(int bank, int pin) {
  // All the failures here are fatal, seeing as
  // if we can't initialize the pin, we're
  // probably screwed anyway.
  if (!InitPin(bank, pin)) {
    LOG(FATAL, "Failed to export pin.\n");
  }
  if (!DoPinDirSet(1)) {
    LOG(FATAL, "Failed to set pin as an input.\n");
  }
}

int Gpi::Read() {
  // NOTE: I can't find any docs confirming that one can indeed
  // poll a pin's value using this method, but it seems that it
  // should work. Really, the "right" (interrupt driven) way to
  // do this is to set an event, but that involves messing with
  // dev tree crap, which I'm not willing to do unless we need
  // this functionality.
  char buf, val_path[64];
  snprintf(val_path, sizeof(val_path), "/sys/class/gpio/gpio%d/value",
           kernel_pin_);

  if ((handle_ = fopen(val_path, "rb")) == NULL) {
    LOG(ERROR, "Unable to open file for pin value reading.\n");
    fclose(handle_);
    return -1;
  }

  if (fread(&buf, sizeof(char), 1, handle_) <= 1) {
    LOG(ERROR, "Reading from file failed with error %d\n",
        ferror(handle_));
    fclose(handle_);
    return -1;
  }
  fclose(handle_);
  return atoi(&buf);
}

} // namespace bbb
