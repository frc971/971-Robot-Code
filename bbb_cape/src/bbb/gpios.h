#ifndef BBB_CAPE_SRC_BBB_GPIOS_H_
#define BBB_CAPE_SRC_BBB_GPIOS_H_

#include <stdio.h>

#include "aos/common/macros.h"

// Controlling GPIO pins
// from C++ is a pain. This code provides a simple wrapper that makes it easy to
// use cleanly.

// Based on example from
// <http://learnbuildshare.wordpress.com/2013/05/29/beaglebone-black-digital-ouput/>

// This is a base class for all gpio related stuff.
// Use either the Gpi or the Gpo subclass if you want to do things.
namespace bbb {

class GpioPin {
 protected:
  // initial_value only matters if input is false.
  GpioPin(int bank, int pin, bool input, bool initial_value = false);
  virtual ~GpioPin();

  FILE *value_handle_ = NULL;
  const int bank_, pin_, kernel_pin_;

 private:
  DISALLOW_COPY_AND_ASSIGN(GpioPin);
};

}  // namespace bbb

#endif  // BBB_CAPE_SRC_BBB_GPIOS_H_
