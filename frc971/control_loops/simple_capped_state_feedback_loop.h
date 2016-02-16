#ifndef FRC971_CONTROL_LOOPS_SIMPLE_CAPPED_STATE_FEEDBACK_LOOP_H_
#define FRC971_CONTROL_LOOPS_SIMPLE_CAPPED_STATE_FEEDBACK_LOOP_H_

#include "Eigen/Dense"

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

// A StateFeedbackLoop which implements CapU based on a simple set of maximum
// absolute values for each element of U.
template <int number_of_states, int number_of_inputs, int number_of_outputs>
class SimpleCappedStateFeedbackLoop
    : public StateFeedbackLoop<number_of_states, number_of_inputs,
                               number_of_outputs> {
 public:
  SimpleCappedStateFeedbackLoop(StateFeedbackLoop<
      number_of_states, number_of_inputs, number_of_outputs> &&loop)
      : StateFeedbackLoop<number_of_states, number_of_inputs,
                          number_of_outputs>(::std::move(loop)),
        min_voltages_(
            ::Eigen::Array<double, number_of_inputs, 1>::Constant(-12)),
        max_voltages_(
            ::Eigen::Array<double, number_of_inputs, 1>::Constant(12)) {}

  void set_max_voltages(
      const ::Eigen::Array<double, number_of_inputs, 1> &max_voltages) {
    max_voltages_ = max_voltages;
    min_voltages_ = -max_voltages;
  }
  void set_max_voltage(int i, double max_voltage) {
    mutable_max_voltage(i) = max_voltage;
    mutable_min_voltage(i) = -max_voltage;
  }

  void set_asymetric_voltages(
      const ::Eigen::Array<double, number_of_inputs, 1> &min_voltages,
      const ::Eigen::Array<double, number_of_inputs, 1> &max_voltages) {
    min_voltages_ = min_voltages;
    max_voltages_ = max_voltages;
  }
  void set_asymetric_voltage(int i, double min_voltage, double max_voltage) {
    mutable_min_voltage(i) = min_voltage;
    mutable_max_voltage(i) = max_voltage;
  }

  const ::Eigen::Array<double, number_of_inputs, 1> &min_voltages() const {
    return min_voltages_;
  }
  ::Eigen::Array<double, number_of_inputs, 1> &mutable_min_voltages() {
    return min_voltages_;
  }
  const ::Eigen::Array<double, number_of_inputs, 1> &max_voltages() const {
    return max_voltages_;
  }
  ::Eigen::Array<double, number_of_inputs, 1> &mutable_max_voltages() {
    return max_voltages_;
  }

  double min_voltage(int i) const { return min_voltages()(i, 0); }
  double &mutable_min_voltage(int i) { return mutable_min_voltages()(i, 0); }

  double max_voltage(int i) const { return max_voltages()(i, 0); }
  double &mutable_max_voltage(int i) { return mutable_max_voltages()(i, 0); }

 private:
  void CapU() override {
    this->mutable_U() =
        this->U().array().min(max_voltages_).max(min_voltages_);
  }

  ::Eigen::Array<double, number_of_inputs, 1> min_voltages_;
  ::Eigen::Array<double, number_of_inputs, 1> max_voltages_;
};

}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_SIMPLE_CAPPED_STATE_FEEDBACK_LOOP_H_
