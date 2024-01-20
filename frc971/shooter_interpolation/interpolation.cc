#include "frc971/shooter_interpolation/interpolation.h"

#include <algorithm>
#include <utility>
#include <vector>

namespace frc971::shooter_interpolation {

double Blend(double coefficient, double a1, double a2) {
  return (1 - coefficient) * a1 + coefficient * a2;
}

}  // namespace frc971::shooter_interpolation
