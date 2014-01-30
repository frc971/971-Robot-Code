#ifndef BBB_CAPE_SRC_BBB_GPI_H_
#define BBB_CAPE_SRC_BBB_GPI_H_

#include "bbb/gpios.h"

namespace bbb {

// Subclass of Pin for input pins.
class Gpi : public GpioPin {
 public:
  Gpi(int bank, int pin);

  // Read the current value of the pin.
  // Returns true if it's high and false if it's low.
  bool Read();
};

}  // namespace bbb

#endif
