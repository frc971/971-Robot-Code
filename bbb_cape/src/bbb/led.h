#ifndef BBB_CAPE_SRC_BBB_LED_H_
#define BBB_CAPE_SRC_BBB_LED_H_

#include <stdio.h>

#include "aos/common/macros.h"

namespace bbb {

// Allows easily controlling one of the LEDs.
class LED {
 public:
  LED(int number);
  ~LED();

  void Set(bool on);

 private:
  FILE *brightness_handle_ = nullptr;
  const int number_;

  DISALLOW_COPY_AND_ASSIGN(LED);
};

}  // namespace bbb

#endif  // BBB_CAPE_SRC_BBB_LED_H_
