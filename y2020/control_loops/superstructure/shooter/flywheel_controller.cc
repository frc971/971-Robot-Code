#include "y2020/control_loops/superstructure/shooter/flywheel_controller.h"

#include <chrono>

#include "aos/logging/logging.h"
#include "y2020/control_loops/superstructure/accelerator/accelerator_plant.h"
#include "y2020/control_loops/superstructure/finisher/finisher_plant.h"

namespace y2020 {
namespace control_loops {
namespace superstructure {
namespace shooter {

// Class to current limit battery current for a flywheel controller.
class CurrentLimitedStateFeedbackController
    : public StateFeedbackLoop<3, 1, 1, double,
                               StateFeedbackHybridPlant<3, 1, 1>,
                               HybridKalman<3, 1, 1>> {
 public:
  // Builds a CurrentLimitedStateFeedbackController given the coefficients, bemf
  // coefficient (units of radians/sec / volt), and motor resistance in ohms.
  CurrentLimitedStateFeedbackController(
      StateFeedbackLoop<3, 1, 1, double, StateFeedbackHybridPlant<3, 1, 1>,
                        HybridKalman<3, 1, 1>> &&other,
      double bemf, double resistance)
      : StateFeedbackLoop(std::move(other)),
        bemf_(bemf),
        resistance_(resistance) {}

  double resistance() const { return resistance_; }
  double bemf() const { return bemf_; }

  void CapU() override {
    const double bemf_voltage = X_hat(1) / bemf_;
    // Solve the system of equations:
    //
    //   motor_current = (u - bemf_voltage) / resistance
    //   battery_current = ((u - bemf_voltage) / resistance) * u / 12.0
    //   0.0 = u * u - u * bemf_voltage - max_current * 12.0 * resistance
    //
    // And we have a quadratic!
    const double a = 1;
    const double b = -bemf_voltage;
    const double c_positive = -70.0 * 12.0 * resistance_;
    const double c_negative = -40.0 * 12.0 * resistance_;

    // Root is always positive.
    const double root_positive = std::sqrt(b * b - 4.0 * a * c_positive);
    const double root_negative = std::sqrt(b * b - 4.0 * a * c_negative);
    const double upper_limit = (-b + root_positive) / (2.0 * a);
    const double lower_limit = (-b - root_negative) / (2.0 * a);

    // Limit to the battery voltage and the current limit voltage.
    mutable_U(0, 0) = std::clamp(U(0, 0), lower_limit, upper_limit);
    if (R(0) > 50.0) {
      mutable_U(0, 0) = std::clamp(U(0, 0), -0.8, 12.0);
    } else {
      mutable_U(0, 0) = std::clamp(U(0, 0), 0.0, 12.0);
    }
  }

 private:
  double bemf_ = 0.0;
  double resistance_ = 0.0;
};

FlywheelController::FlywheelController(
    StateFeedbackLoop<3, 1, 1, double, StateFeedbackHybridPlant<3, 1, 1>,
                      HybridKalman<3, 1, 1>> &&loop,
    double bemf, double resistance)
    : loop_(new CurrentLimitedStateFeedbackController(std::move(loop), bemf,
                                                      resistance)) {
  history_.fill(std::pair<double, ::aos::monotonic_clock::time_point>(
      0, ::aos::monotonic_clock::epoch()));
  Y_.setZero();
}

void FlywheelController::set_goal(double angular_velocity_goal) {
  loop_->mutable_next_R() << 0.0, angular_velocity_goal, 0.0;
  last_goal_ = angular_velocity_goal;
}

void FlywheelController::set_position(
    double current_position,
    const aos::monotonic_clock::time_point position_timestamp) {
  // Project time forwards.
  const int newest_history_position =
      ((history_position_ == 0) ? kHistoryLength : history_position_) - 1;

  if (!first_) {
    loop_->UpdateObserver(
        loop_->U(),
        position_timestamp - std::get<1>(history_[newest_history_position]));
  } else {
    first_ = false;
  }

  // Update position in the model.
  Y_ << current_position;

  // Add the position to the history.
  history_[history_position_] =
      std::pair<double, ::aos::monotonic_clock::time_point>(current_position,
                                                            position_timestamp);
  history_position_ = (history_position_ + 1) % kHistoryLength;

  loop_->Correct(Y_);
}

double FlywheelController::voltage() const { return loop_->U(0, 0); }

double FlywheelController::current() const {
  return ((voltage() - (velocity() / loop_->bemf())) / (loop_->resistance())) *
         voltage() / 12.0;
}

void FlywheelController::Update(bool disabled) {
  loop_->mutable_R() = loop_->next_R();
  if (loop_->R(1, 0) < 1.0) {
    // Kill power at low angular velocities.
    disabled = true;
  }

  loop_->UpdateController(disabled);
}

flatbuffers::Offset<FlywheelControllerStatus> FlywheelController::SetStatus(
    flatbuffers::FlatBufferBuilder *fbb) {
  // Compute the oldest point in the history.
  const int oldest_history_position = history_position_;
  const int newest_history_position =
      ((history_position_ == 0) ? kHistoryLength : history_position_) - 1;
  const int second_newest_history_position =
      ((newest_history_position == 0) ? kHistoryLength
                                      : newest_history_position) -
      1;

  const double total_loop_time = ::aos::time::DurationInSeconds(
      std::get<1>(history_[newest_history_position]) -
      std::get<1>(history_[oldest_history_position]));

  const double distance_traveled =
      std::get<0>(history_[newest_history_position]) -
      std::get<0>(history_[oldest_history_position]);

  const double last_loop_time = ::aos::time::DurationInSeconds(
      std::get<1>(history_[newest_history_position]) -
      std::get<1>(history_[second_newest_history_position]));

  const double last_distance_traveled =
      std::get<0>(history_[newest_history_position]) -
      std::get<0>(history_[second_newest_history_position]);

  // Compute the distance moved over that time period.
  avg_angular_velocity_ = (distance_traveled) / (total_loop_time);

  FlywheelControllerStatusBuilder builder(*fbb);

  builder.add_avg_angular_velocity(avg_angular_velocity_);
  builder.add_dt_angular_velocity(last_distance_traveled / last_loop_time);
  builder.add_angular_velocity(loop_->X_hat(1, 0));
  builder.add_voltage_error(loop_->X_hat(2, 0));
  builder.add_commanded_current(current());
  builder.add_angular_velocity_goal(last_goal_);
  return builder.Finish();
}

FlywheelController::~FlywheelController() {}

double FlywheelController::velocity() const { return loop_->X_hat(1, 0); }

}  // namespace shooter
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2020
