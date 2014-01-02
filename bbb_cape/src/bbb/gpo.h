#ifndef BBB_CAPE_SRC_BBB_GPO_H_
#define BBB_CAPE_SRC_BBB_GPO_H_

#include "gpios.h"

namespace bbb {

// Gpios subclass for output pins.
class Gpo : public Pin {
 public:
  // See the base class for what these args mean.
  Gpo(int bank, int pin);

  // Methods set the pin to read either high or low.
  inline bool SetHigh() {
    return DoSet(1);
  }
  inline bool SetLow() {
    return DoSet(0);
  }

 private:
  // Does the actual pin setting.
  bool DoSet(const int value);
};

} // namespace bbb

#endif
