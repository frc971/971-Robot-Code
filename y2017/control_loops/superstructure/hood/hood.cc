#include "y2017/control_loops/superstructure/hood/hood.h"

#include "y2017/constants.h"
#include "y2017/control_loops/superstructure/hood/hood_integral_plant.h"

namespace y2017 {
namespace control_loops {
namespace superstructure {
namespace hood {

constexpr double Hood::kZeroingVoltage;
constexpr double Hood::kOperatingVoltage;

Hood::Hood()
    : profiled_subsystem_(
          ::std::unique_ptr<
              ::frc971::control_loops::SimpleCappedStateFeedbackLoop<3, 1, 1>>(
              new ::frc971::control_loops::SimpleCappedStateFeedbackLoop<
                  3, 1, 1>(MakeIntegralHoodLoop())),
          constants::GetValues().hood.zeroing, constants::Values::kHoodRange,
          0.5, 10.0) {}

void Hood::Reset() {
  state_ = State::UNINITIALIZED;
  profiled_subsystem_.Reset();
}

void Hood::Iterate(const control_loops::HoodGoal *unsafe_goal,
                   const ::frc971::PotAndIndexPosition *position,
                   double *output,
                   ::frc971::control_loops::ProfiledJointStatus *status) {
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
      profiled_subsystem_.AdjustProfile(0.10, 1);

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
        // Seek between the two soft limits.
        if (profiled_subsystem_.position() >
            (profiled_subsystem_.range().lower +
             profiled_subsystem_.range().upper) /
                2.0) {
          // We are above the middle.
          if (profiled_subsystem_.goal(1, 0) > 0) {
            // And moving up.  Keep going to the upper soft limit until we
            // arrive.
            profiled_subsystem_.set_unprofiled_goal(
                profiled_subsystem_.range().upper);
          } else {
            // And no longer moving.  Go down to the lower soft limit.
            profiled_subsystem_.set_unprofiled_goal(
                profiled_subsystem_.range().lower);
          }
        } else {
          // We are below the middle.
          if (profiled_subsystem_.goal(1, 0) < 0) {
            // And moving down.  Keep going to the lower soft limit until we
            // arrive.
            profiled_subsystem_.set_unprofiled_goal(
                profiled_subsystem_.range().lower);
          } else {
            // And no longer moving.  Go up to the upper soft limit.
            profiled_subsystem_.set_unprofiled_goal(
                profiled_subsystem_.range().upper);
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
      if (profiled_subsystem_.CheckHardLimits()) {
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

  profiled_subsystem_.set_max_voltage({{max_voltage}});

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
