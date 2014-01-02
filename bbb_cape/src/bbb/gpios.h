#ifndef BBB_CAPE_SRC_BBB_CAPE_CONTROL_H_
#define BBB_CAPE_SRC_BBB_CAPE_CONTROL_H_

#include <stdint.h>
#include <stdio.h>

// As it turns out, controlling the BBB's GPIO pins
// from C++ is kind of a pain. The purpose of this
// code is to provide a simple wrapper that masks
// all the ugly stuff and exposes a nice API.

// Based on example from 
// <http://learnbuildshare.wordpress.com/2013/05/29/beaglebone-black-digital-ouput/>

namespace bbb {

class Pin {
  FILE *handle_;
  int direction_;
  int kernel_pin_;
  bool exported_;

  int DoPinDirSet(int direction);
  int DoExport();

public:
  // Here, for example, if you wanted to use
  // GPIO1_28, you would give the ctor
  // 1, 28 as arguments.
  Pin(int bank, int pin);
  ~Pin();
  int MakeInput();
  int MakeOutput();
  int Write(uint8_t value);
  int Read();
};

}  // namespace bbb

#endif
