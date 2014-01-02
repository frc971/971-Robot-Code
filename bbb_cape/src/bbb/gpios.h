#ifndef BBB_CAPE_SRC_BBB_CAPE_CONTROL_H_
#define BBB_CAPE_SRC_BBB_CAPE_CONTROL_H_

#include <stdio.h>

// As it turns out, controlling the BBB's GPIO pins
// from C++ is kind of a pain. The purpose of this
// code is to provide a simple wrapper that masks
// all the ugly stuff and exposes a nice API.

// Based on example from
// <http://learnbuildshare.wordpress.com/2013/05/29/beaglebone-black-digital-ouput/>

// This is a base class for all gpio related stuff.
// Use either a gpi or gpo subclass if you want to do things.
namespace bbb {

class Pin {
 public:
  // Not strictly necessary for this to be virtual,
  // but it's a good idea.
  virtual ~Pin();

 protected:
  // Set the pin direction.
  // 1 makes it an input.
  // 2 makes it an output and sets the initial state to low.
  bool DoPinDirSet(int direction);
  // Export the pin, so we can use it.
  // Here, for example, if you wanted to use
  // GPIO1_28, you would give it 1, 28 as arguments.
  bool InitPin(int bank, int pin);

  FILE *handle_ = NULL;
  int kernel_pin_ = -1;
};

}  // namespace bbb

#endif
