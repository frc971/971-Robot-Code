#ifndef FRC971_CONTROL_LOOPS_FLYWHEEL_FLYWHEEL_TEST_PLANT_H_
#define FRC971_CONTROL_LOOPS_FLYWHEEL_FLYWHEEL_TEST_PLANT_H_

#include "frc971/control_loops/flywheel/flywheel_controller.h"

namespace frc971 {
namespace control_loops {
namespace flywheel {

class FlywheelPlant : public StateFeedbackPlant<2, 1, 1> {
 public:
  explicit FlywheelPlant(StateFeedbackPlant<2, 1, 1> &&other, double bemf,
                         double resistance)
      : StateFeedbackPlant<2, 1, 1>(::std::move(other)),
        bemf_(bemf),
        resistance_(resistance) {}

  void CheckU(const Eigen::Matrix<double, 1, 1> &U) override {
    EXPECT_LE(U(0, 0), U_max(0, 0) + 0.00001 + voltage_offset_);
    EXPECT_GE(U(0, 0), U_min(0, 0) - 0.00001 + voltage_offset_);
  }

  double motor_current(const Eigen::Matrix<double, 1, 1> U) const {
    return (U(0) - X(1) / bemf_) / resistance_;
  }

  double battery_current(const Eigen::Matrix<double, 1, 1> U) const {
    return motor_current(U) * U(0) / 12.0;
  }

  double voltage_offset() const { return voltage_offset_; }
  void set_voltage_offset(double voltage_offset) {
    voltage_offset_ = voltage_offset;
  }

 private:
  double voltage_offset_ = 0.0;

  double bemf_;
  double resistance_;
};

}  // namespace flywheel
}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_FLYWHEEL_FLYWHEEL_TEST_PLANT_H_
