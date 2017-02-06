#include "frc971/control_loops/profiled_subsystem.h"

namespace frc971 {
namespace control_loops {
namespace internal {

double UseUnlessZero(double target_value, double default_value) {
  if (target_value != 0.0) {
    return target_value;
  } else {
    return default_value;
  }
}

}  // namespace internal
}  // namespace control_loops
}  // namespace frc971
