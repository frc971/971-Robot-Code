#include "y2017/control_loops/superstructure/hood/hood.h"

#include "y2017/constants.h"
#include "y2017/control_loops/superstructure/hood/hood_integral_plant.h"

namespace y2017 {
namespace control_loops {
namespace superstructure {
namespace hood {

namespace chrono = ::std::chrono;

constexpr double Hood::kZeroingVoltage;
constexpr double Hood::kOperatingVoltage;
constexpr ::aos::monotonic_clock::duration Hood::kTimeTillNotMoving;

// The tracking error to allow before declaring that we are stuck and reversing
// direction while zeroing.
constexpr double kStuckZeroingTrackingError = 0.02;

IndexPulseProfiledSubsystem::IndexPulseProfiledSubsystem()
    : ::frc971::control_loops::SingleDOFProfiledSubsystem<
          ::frc971::zeroing::PulseIndexZeroingEstimator>(
          ::std::unique_ptr<
              ::frc971::control_loops::SimpleCappedStateFeedbackLoop<3, 1, 1>>(
              new ::frc971::control_loops::SimpleCappedStateFeedbackLoop<
                  3, 1, 1>(MakeIntegralHoodLoop())),
          constants::GetValues().hood.zeroing, constants::Values::kHoodRange,
          0.5, 10.0) {}

void IndexPulseProfiledSubsystem::CapGoal(const char *name,
                                          Eigen::Matrix<double, 3, 1> *goal) {
  if (zeroed()) {
    ::frc971::control_loops::SingleDOFProfiledSubsystem<
        ::frc971::zeroing::PulseIndexZeroingEstimator>::CapGoal(name, goal);
  } else {
    const double kMaxRange = range().upper_hard - range().lower_hard;
    // Limit the goal to min/max allowable positions much less agressively.
    // We don't know where the limits are, so we have to let the user move far
    // enough to find them (and the index pulse which might be right next to
    // one).
    if ((*goal)(0, 0) > kMaxRange) {
      LOG(WARNING, "Goal %s above limit, %f > %f\n", name, (*goal)(0, 0),
          kMaxRange);
      (*goal)(0, 0) = kMaxRange;
    }
    if ((*goal)(0, 0) < -kMaxRange) {
      LOG(WARNING, "Goal %s below limit, %f < %f\n", name, (*goal)(0, 0),
          kMaxRange);
      (*goal)(0, 0) = -kMaxRange;
    }
  }
}

Hood::Hood() {}

void Hood::Reset() {
  state_ = State::UNINITIALIZED;
  profiled_subsystem_.Reset();
  last_move_time_ = ::aos::monotonic_clock::min_time;
  last_position_ = 0;
}

void Hood::Iterate(const control_loops::HoodGoal *unsafe_goal,
                   const ::frc971::IndexPosition *position, double *output,
                   ::frc971::control_loops::IndexProfiledJointStatus *status) {
  bool disable = output == nullptr;
  profiled_subsystem_.Correct(*position);

  switch (state_) {
    case State::UNINITIALIZED:
      // Wait in the uninitialized state until the hood is initialized.
      LOG(DEBUG, "Uninitialized, waiting for hood\n");
      if (profiled_subsystem_.initialized()) {
        state_ = State::DISABLED_INITIALIZED;
      }
      disable = true;
      break;

    case State::DISABLED_INITIALIZED:
      // Wait here until we are either fully zeroed while disabled, or we become
      // enabled.
      if (disable) {
        if (profiled_subsystem_.zeroed()) {
          state_ = State::RUNNING;
        }
      } else {
        state_ = State::ZEROING;
      }

      // Set the goals to where we are now so when we start back up, we don't
      // jump.
      profiled_subsystem_.ForceGoal(profiled_subsystem_.position());
      // Set up the profile to be the zeroing profile.
      profiled_subsystem_.AdjustProfile(0.30, 1.0);

      // We are not ready to start doing anything yet.
      disable = true;
      break;

    case State::ZEROING:
      if (profiled_subsystem_.zeroed()) {
        // Move the goal to the current goal so we stop moving.
        profiled_subsystem_.set_unprofiled_goal(profiled_subsystem_.goal(0, 0));
        state_ = State::RUNNING;
      } else if (disable) {
        state_ = State::DISABLED_INITIALIZED;
      } else {
        const double kRange = profiled_subsystem_.range().upper_hard -
                              profiled_subsystem_.range().lower_hard;
        // Seek +- the range of motion.
        if (profiled_subsystem_.position() > 0) {
          // We are above the middle.
          if (profiled_subsystem_.goal(1, 0) > 0 &&
              ::std::abs(profiled_subsystem_.position() -
                         profiled_subsystem_.goal(0, 0)) <
                  kStuckZeroingTrackingError) {
            // And moving up and not stuck.  Keep going until we've gone the
            // full range of motion or get stuck.
            profiled_subsystem_.set_unprofiled_goal(kRange);
          } else {
            // And no longer moving.  Go down to the opposite of the range of
            // motion.
            profiled_subsystem_.set_unprofiled_goal(-kRange);
          }
        } else {
          // We are below the middle.
          if (profiled_subsystem_.goal(1, 0) < 0 &&
              ::std::abs(profiled_subsystem_.position() -
                         profiled_subsystem_.goal(0, 0)) <
                  kStuckZeroingTrackingError) {
            // And moving down and not stuck.  Keep going until we've gone the
            // full range of motion or get stuck.
            profiled_subsystem_.set_unprofiled_goal(-kRange);
          } else {
            // And no longer moving.  Go up to the opposite of the range of
            // motion.
            profiled_subsystem_.set_unprofiled_goal(kRange);
          }
        }
      }
      break;

    case State::RUNNING: {
      if (disable) {
        // Reset the profile to the current position so it starts from here when
        // we get re-enabled.
        profiled_subsystem_.ForceGoal(profiled_subsystem_.position());
      }

      // If we have a goal, go to it.  Otherwise stay where we are.
      if (unsafe_goal) {
        profiled_subsystem_.AdjustProfile(unsafe_goal->profile_params);
        profiled_subsystem_.set_unprofiled_goal(unsafe_goal->angle);
      }

      // ESTOP if we hit the hard limits.
      if (profiled_subsystem_.CheckHardLimits() ||
          profiled_subsystem_.error()) {
        state_ = State::ESTOP;
      }
    } break;

    case State::ESTOP:
      LOG(ERROR, "Estop\n");
      disable = true;
      break;
  }

  // Set the voltage limits.
  const double max_voltage =
      (state_ == State::RUNNING) ? kOperatingVoltage : kZeroingVoltage;

  // If we have been in the same position for kNumberCyclesTillNotMoving, make
  // sure that the kNotMovingVoltage is used instead of kOperatingVoltage.
  double error_voltage = max_voltage;
  if (::std::abs(profiled_subsystem_.position() - last_position_) >
      kErrorOnPositionTillNotMoving) {
    // Currently moving. Update time of last move.
    last_move_time_ = ::aos::monotonic_clock::now();
    // Save last position.
    last_position_ = profiled_subsystem_.position();
  }
  if (::aos::monotonic_clock::now() > kTimeTillNotMoving + last_move_time_) {
    error_voltage = kNotMovingVoltage;
  }

  profiled_subsystem_.set_max_voltage(
      {{::std::min(max_voltage, error_voltage)}});

  // Calculate the loops for a cycle.
  profiled_subsystem_.Update(disable);

  // Write out all the voltages.
  if (output) {
    *output = profiled_subsystem_.voltage();
  }

  // Save debug/internal state.
  // TODO(austin): Save more.
  status->zeroed = profiled_subsystem_.zeroed();

  profiled_subsystem_.PopulateStatus(status);
  status->estopped = (state_ == State::ESTOP);
  status->state = static_cast<int32_t>(state_);
}

}  // namespace hood
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2017
