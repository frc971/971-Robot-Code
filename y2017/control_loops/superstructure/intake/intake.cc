#include "y2017/control_loops/superstructure/intake/intake.h"

#include "y2017/constants.h"
#include "y2017/control_loops/superstructure/intake/intake_integral_plant.h"

namespace y2017 {
namespace control_loops {
namespace superstructure {
namespace intake {

constexpr double Intake::kZeroingVoltage;
constexpr double Intake::kOperatingVoltage;

Intake::Intake()
    : profiled_subsystem_(
          ::std::unique_ptr<
              ::frc971::control_loops::SimpleCappedStateFeedbackLoop<3, 1, 1>>(
              new ::frc971::control_loops::SimpleCappedStateFeedbackLoop<
                  3, 1, 1>(MakeIntegralIntakeLoop())),
          constants::GetValues().intake.zeroing,
          constants::Values::kIntakeRange, 0.3, 5.0) {
  clear_min_position();
}

void Intake::Reset() {
  state_ = State::UNINITIALIZED;
  clear_min_position();
  profiled_subsystem_.Reset();
  disable_count_ = 0;
}

void Intake::Iterate(
    const control_loops::IntakeGoal *unsafe_goal,
    const ::frc971::PotAndAbsolutePosition *position, double *output,
    ::frc971::control_loops::AbsoluteProfiledJointStatus *status) {
  bool disable = output == nullptr;
  profiled_subsystem_.Correct(*position);

  switch (state_) {
    case State::UNINITIALIZED:
      // Wait in the uninitialized state until the intake is initialized.
      LOG(DEBUG, "Uninitialized, waiting for intake\n");
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
      // Now, zero by actively holding still.
      if (profiled_subsystem_.zeroed()) {
        state_ = State::RUNNING;
      } else if (disable) {
        state_ = State::DISABLED_INITIALIZED;
      }
      break;

    case State::RUNNING: {
      if (disable) {
        // Reset the profile to the current position so it starts from here when
        // we get re-enabled.
        profiled_subsystem_.ForceGoal(profiled_subsystem_.position());
      }

      if (unsafe_goal) {
        profiled_subsystem_.AdjustProfile(unsafe_goal->profile_params);
        profiled_subsystem_.set_unprofiled_goal(unsafe_goal->distance);
        if (unsafe_goal->disable_intake) {
          if (disable_count_ > 100) {
            disable = true;
          }
          ++disable_count_;
          if (disable_count_ > 200) {
            state_ = State::ESTOP;
          }
        } else {
          disable_count_ = 0;
        }
      }

      // Force the intake to be at least at min_position_ out.
      if (profiled_subsystem_.unprofiled_goal(0, 0) < min_position_) {
        LOG(INFO, "Limiting intake to %f from %f\n", min_position_,
            profiled_subsystem_.unprofiled_goal(0, 0));
        profiled_subsystem_.set_unprofiled_goal(min_position_);
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

}  // namespace intake
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2017
