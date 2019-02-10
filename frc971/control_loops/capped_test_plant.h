#ifndef FRC971_CONTROL_LOOPS_CAPPED_TEST_PLANT_H_
#define FRC971_CONTROL_LOOPS_CAPPED_TEST_PLANT_H_

#include "frc971/control_loops/state_feedback_loop.h"
#include "gtest/gtest.h"

namespace frc971 {
namespace control_loops {

// Basic state feedback plant for use in tests.
class CappedTestPlant : public StateFeedbackPlant<2, 1, 1> {
 public:
  explicit CappedTestPlant(StateFeedbackPlant<2, 1, 1> &&other)
      : StateFeedbackPlant<2, 1, 1>(::std::move(other)) {}

  void CheckU(const ::Eigen::Matrix<double, 1, 1> &U) override {
    EXPECT_LE(U(0, 0), U_max(0, 0) + 0.00001 + voltage_offset_);
    EXPECT_GE(U(0, 0), U_min(0, 0) - 0.00001 + voltage_offset_);
  }

  double voltage_offset() const { return voltage_offset_; }
  void set_voltage_offset(double voltage_offset) {
    voltage_offset_ = voltage_offset;
  }

 private:
  double voltage_offset_ = 0.0;
};

}  // namespace frc971
}  // namespace control_loops
#endif  // FRC971_CONTROL_LOOPS_CAPPED_TEST_PLANT_H_