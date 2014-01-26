#ifndef BBB_CAPE_SRC_BBB_GPI_H_
#define BBB_CAPE_SRC_BBB_GPI_H_

#include "gpios.h"

// Subclass of Pin for input pins.

namespace bbb {

class Gpi : public Pin {
 public:
  // See the base class for what these args mean.
  Gpi(int bank, int pin);

  // Read the current value of the pin.
  // Returns 1 if it reads high, 0 for low.
  // Returns -1 if fails to read the pin for some reason.
  int Read();
};

}  // namespace bbb

#endif
