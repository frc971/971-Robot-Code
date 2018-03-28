#include "frc971/zeroing/wrap.h"

#include <cmath>

namespace frc971 {
namespace zeroing {

float Wrap(float nearest, float value, float period) {
  return remainderf(value - nearest, period) + nearest;
}

double Wrap(double nearest, double value, double period) {
  return remainder(value - nearest, period) + nearest;
}

}  // namespace zeroing
}  // namespace frc971
