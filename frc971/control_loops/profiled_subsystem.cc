#include "frc971/control_loops/profiled_subsystem.h"

namespace frc971::control_loops::internal {

double UseUnlessZero(double target_value, double default_value) {
  if (target_value != 0.0) {
    return target_value;
  } else {
    return default_value;
  }
}

}  // namespace frc971::control_loops::internal
