#include "frc971/zeroing/wrap.h"

#include <cmath>

namespace frc971 {
namespace zeroing {

double Wrap(double nearest, double value, double period) {
  return ::std::remainder(value - nearest, period) + nearest;
}

}  // namespace zeroing
}  // namespace frc971
