#include "bbb/gpios.h"

#include <stdio.h>
#include <stdlib.h>

#include "aos/common/logging/logging_impl.h"

namespace bbb {

Pin::Pin(int bank, int pin)
    : handle_(NULL),
      direction_(0),
      kernel_pin_(bank * 32 + pin),
      exported_(false) {}

Pin::~Pin() {
  // Unexport the pin.
  if ((handle_ = fopen("/sys/class/gpio/unexport", "ab")) == NULL) {
    LOG(WARNING, "Could not unexport gpio pin.\n");
    // There's nothing intelligent we can really do here.
    return;
  }

  char gpio_num[2];
  snprintf(gpio_num, sizeof(gpio_num), "%d", kernel_pin_);
  fwrite(gpio_num, sizeof(char), 2, handle_);
  fclose(handle_);
}

int Pin::DoExport() {
  char gpio_num[2];
  snprintf(gpio_num, sizeof(gpio_num), "%d", kernel_pin_);

  // Export the pin.
  if ((handle_ = fopen("/sys/class/gpio/export", "ab")) == NULL) {
    LOG(ERROR, "Could not export GPIO pin.\n");
    return -1;
  }

  fwrite(gpio_num, sizeof(char), 2, handle_);
  fclose(handle_);

  exported_ = true;
  return 0;
}

int Pin::DoPinDirSet(int direction) {
  char buf[4], type_path[64];
  snprintf(type_path, sizeof(type_path), "/sys/class/gpio/gpio%d/direction",
           kernel_pin_);

  if ((handle_ = fopen(type_path, "rb+")) == NULL) {
    LOG(ERROR, "Unable to set pin direction.\n");
    return -1;
  }

  direction_ = direction;
  switch (direction) {
    case 1:
      strcpy(buf, "in");
      break;
    case 2:
      strcpy(buf, "low");
      break;
    case 0:
      return 0;
    default:
      LOG(ERROR, "Invalid direction identifier %d.\n", direction);
      direction_ = 0;
      return -1;
  }
  fwrite(buf, sizeof(char), 3, handle_);
  fclose(handle_);

  return 0;
}

int Pin::MakeInput() {
  if (!exported_) {
    if (DoExport()) {
      return -1;
    }
  }

  return DoPinDirSet(1);
}

int Pin::MakeOutput() {
  if (!exported_) {
    if (DoExport()) {
      return -1;
    }
  }

  return DoPinDirSet(2);
}

int Pin::Write(uint8_t value) {
  if (direction_ != 2) {
    LOG(ERROR, "Only pins set as output can be written to.\n");
    return -1;
  }

  char buf, val_path[64];
  snprintf(val_path, sizeof(val_path), "/sys/class/gpio/gpio%d/value",
           kernel_pin_);
  if (value != 0) {
    value = 1;
  }

  if ((handle_ = fopen(val_path, "rb+")) == NULL) {
    LOG(ERROR, "Unable to set pin value.\n");
    return -1;
  }

  snprintf(&buf, sizeof(buf), "%d", value);
  fwrite(&buf, sizeof(char), 1, handle_);
  fclose(handle_);

  return 0;
}

int Pin::Read() {
  // NOTE: I can't find any docs confirming that one can indeed
  // poll a pin's value using this method, but it seems that it
  // should work. Really, the "right" (interrupt driven) way to
  // do this is to set an event, but that involves messing with
  // dev tree crap, which I'm not willing to do unless we need
  // this functionality.

  if (direction_ != 1) {
    LOG(ERROR, "Only pins set as input can be read from.\n");
    return -1;
  }

  char buf, val_path[64];
  snprintf(val_path, sizeof(val_path), "/sys/class/gpio/gpio%d/value",
           kernel_pin_);

  if ((handle_ = fopen(val_path, "rb")) == NULL) {
    LOG(ERROR, "Unable to read pin value.\n");
    return -1;
  }

  if (fread(&buf, sizeof(char), 1, handle_) <= 0) {
    LOG(ERROR, "Failed to read pin value from file.\n");
    return -1;
  }
  fclose(handle_);
  return atoi(&buf);
}

}  // namespace bbb
