#ifndef BBB_CAPE_SRC_BBB_GPO_H_
#define BBB_CAPE_SRC_BBB_GPO_H_

#include "bbb/gpios.h"

namespace bbb {

// Gpios subclass for output pins.
class Gpo : public GpioPin {
 public:
  Gpo(int bank, int pin, bool initial_value = false);
  // Sets the pin to either high (true) or low (false).
  void Set(bool high);
};

}  // namespace bbb

#endif
