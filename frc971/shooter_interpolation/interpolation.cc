#include "frc971/shooter_interpolation/interpolation.h"

#include <algorithm>
#include <utility>
#include <vector>

namespace frc971 {
namespace shooter_interpolation {

namespace {

double Blend(double coefficient, double a1, double a2) {
  return (1 - coefficient) * a1 + coefficient * a2;
}

ShotParams Blend(double coefficient, ShotParams a1, ShotParams a2) {
  return ShotParams{Blend(coefficient, a1.angle, a2.angle),
                    Blend(coefficient, a1.power, a2.power)};
}

}  // namespace

InterpolationTable::InterpolationTable(
    ::std::vector<::std::pair<double, ShotParams>> interpolation_table) {
  interpolation_table_ = ::std::move(interpolation_table);
  ::std::sort(interpolation_table_.begin(), interpolation_table_.end(),
              [](const ::std::pair<double, ShotParams> &a,
                 const ::std::pair<double, ShotParams> &b) {
    return a.first < b.first;
  });
}

ShotParams InterpolationTable::GetShooterData(double distance) {
  // Points to to the smallest item such that it->first >= dist, or end() if no
  // such item exists.
  auto it =
      std::lower_bound(interpolation_table_.begin(), interpolation_table_.end(),
                       distance, [](const ::std::pair<double, ShotParams> &a,
                                    double dist) { return a.first < dist; });
  if (it == interpolation_table_.begin()) {
    return it->second;
  } else if (it == interpolation_table_.end()) {
    return interpolation_table_.back().second;
  } else {
    auto x_a2 = it;
    auto x_a1 = it - 1;
    double x1 = x_a1->first;
    double x2 = x_a2->first;
    double coefficient = (distance - x1) / (x2 - x1);
    return Blend(coefficient, x_a1->second, x_a2->second);
  }
}

}  // namespace shooter_interpolation
}  // namespace frc971
