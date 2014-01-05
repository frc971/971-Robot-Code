#ifndef BBB_CAPE_SRC_BBB_GPO_H_
#define BBB_CAPE_SRC_BBB_GPO_H_

#include "gpios.h"

namespace bbb {

// Gpios subclass for output pins.
class Gpo : public Pin {
 public:
  // See the base class for what these args mean.
  Gpo(int bank, int pin);
  // Sets the pin to either high or low.
  // If the argument is true, is sets it high.
  // Otherwise, it sets it low.
  bool Set(const bool high);
};

}  // namespace bbb

#endif
