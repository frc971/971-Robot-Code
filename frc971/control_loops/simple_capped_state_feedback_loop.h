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
                          number_of_outputs>(::std::move(loop)) {}

  void set_max_voltages(
      const ::Eigen::Array<double, number_of_inputs, 1> &max_voltages) {
    max_voltages_ = max_voltages;
  }
  void set_max_voltage(int i, double max_voltage) {
    mutable_max_voltage(i) = max_voltage;
  }

  // Easier to use overloads for number_of_inputs == 1 or 2. Using the wrong one
  // will result in a compile-time Eigen error about mixing matrices of
  // different sizes.
  void set_max_voltages(double v1) {
    set_max_voltages((::Eigen::Array<double, 1, 1>() << v1).finished());
  }
  void set_max_voltages(double v1, double v2) {
    set_max_voltages((::Eigen::Array<double, 2, 1>() << v1, v2).finished());
  }

  const ::Eigen::Array<double, number_of_inputs, 1> &max_voltages() const {
    return max_voltages_;
  }
  ::Eigen::Array<double, number_of_inputs, 1> &mutable_max_voltages() {
    return max_voltages_;
  }

  double max_voltage(int i) const { return max_voltages()(i, 0); }
  double max_voltage(double) const = delete;
  double &mutable_max_voltage(int i) { return mutable_max_voltages()(i, 0); }
  double &mutable_max_voltage(double) = delete;

  // Don't accidentally call these when you mean to call set_max_voltages
  // with a low number_of_inputs.
  void set_max_voltage(double) = delete;
  void set_max_voltage(double, double) = delete;

 private:
  void CapU() override {
    this->mutable_U() =
        this->U().array().min(max_voltages_).max(-max_voltages_);
  }

  ::Eigen::Array<double, number_of_inputs, 1> max_voltages_ =
      ::Eigen::Array<double, number_of_inputs, 1>::Constant(12);
};

}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_SIMPLE_CAPPED_STATE_FEEDBACK_LOOP_H_
