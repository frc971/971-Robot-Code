#ifndef FRC971_SHOOTER_INTERPOLATION_INTERPOLATION_H_
#define FRC971_SHOOTER_INTERPOLATION_INTERPOLATION_H_

#include <utility>
#include <vector>

namespace frc971 {
namespace shooter_interpolation {

// Struct for shot angle and power
struct ShotParams {
  double angle;
  double power;
};

class InterpolationTable {
 public:
  InterpolationTable() = default;
  InterpolationTable(
      const ::std::vector<::std::pair<double, ShotParams>> &table);

  // Uses the interpolation table to calculate the optimal shooter angle and
  // power for a shot
  ShotParams GetShooterData(double distance) const;

 private:
  // Contains the list of angle entries in the interpolation table
  ::std::vector<::std::pair<double, ShotParams>> table_;
};

}  // namespace shooter_interpolation
}  // namespace frc971

#endif  // FRC971_SHOOTER_INTERPOLATION_INTERPOLATION_H_
