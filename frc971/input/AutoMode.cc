#include "frc971/input/AutoMode.q.h"

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "aos/common/control_loop/Timing.h"

namespace frc971 {

void AutoMode_t::DoAction() {
  Sleep(10); // wait until it ends
}

} // namespace frc971
