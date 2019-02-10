#include "static_zeroing_single_dof_profiled_subsystem.h"

namespace frc971 {
namespace control_loops {

StaticZeroingSingleDOFProfiledSubsystem::
    StaticZeroingSingleDOFProfiledSubsystem(
        const StaticZeroingSingleDOFProfiledSubsystemParams &params)
    : params_(params),
      profiled_subsystem_(
          ::std::unique_ptr<
              ::frc971::control_loops::SimpleCappedStateFeedbackLoop<3, 1, 1>>(
              new ::frc971::control_loops::SimpleCappedStateFeedbackLoop<
                  3, 1, 1>(params.make_integral_loop())),
          params.zeroing_constants, params.range,
          params.default_profile_params.max_velocity,
          params.default_profile_params.max_acceleration) {
  Reset();
};

void StaticZeroingSingleDOFProfiledSubsystem::Reset() {
  state_ = State::UNINITIALIZED;
  clear_min_position();
  clear_max_position();
  profiled_subsystem_.Reset();
}

void StaticZeroingSingleDOFProfiledSubsystem::Iterate(
    const StaticZeroingSingleDOFProfiledSubsystemGoal *goal,
    const typename zeroing::PotAndAbsoluteEncoderZeroingEstimator::Position
        *position,
    double *output,
    ::frc971::control_loops::AbsoluteProfiledJointStatus *status) {
  bool disabled = output == nullptr;
  profiled_subsystem_.Correct(*position);

  switch (state_) {
    case State::UNINITIALIZED:
      if (profiled_subsystem_.initialized()) {
        state_ = State::DISABLED_INITIALIZED;
      }
      disabled = true;
      break;
    case State::DISABLED_INITIALIZED:
      // Wait here until we are either fully zeroed while disabled, or we become
      // enabled.
      if (disabled) {
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
      profiled_subsystem_.AdjustProfile(params_.zeroing_profile_params);

      // We are not ready to start doing anything yet.
      disabled = true;
      break;
    case State::ZEROING:
      // Now, zero by actively holding still.
      if (profiled_subsystem_.zeroed()) {
        state_ = State::RUNNING;
      } else if (disabled) {
        state_ = State::DISABLED_INITIALIZED;
      }
      break;

    case State::RUNNING: {
      if (disabled) {
        // Reset the profile to the current position so it starts from here when
        // we get re-enabled.
        profiled_subsystem_.ForceGoal(profiled_subsystem_.position());
      }

      if (goal) {
        profiled_subsystem_.AdjustProfile(goal->profile_params);

        double safe_goal = goal->unsafe_goal;
        if (safe_goal < min_position_) {
          LOG(DEBUG, "Limiting to %f from %f\n", min_position_, safe_goal);
          safe_goal = min_position_;
        }
        if (safe_goal > max_position_) {
          LOG(DEBUG, "Limiting to %f from %f\n", max_position_, safe_goal);
          safe_goal = max_position_;
        }
        profiled_subsystem_.set_unprofiled_goal(safe_goal);
      }
    } break;

    case State::ESTOP:
      LOG(ERROR, "Estop\n");
      disabled = true;
      break;
  }

  // Set the voltage limits.
  const double max_voltage = (state_ == State::RUNNING)
                                 ? params_.operating_voltage
                                 : params_.zeroing_voltage;

  profiled_subsystem_.set_max_voltage({{max_voltage}});

  // Calculate the loops for a cycle.
  profiled_subsystem_.Update(disabled);

  // Write out all the voltages.
  if (output) {
    *output = profiled_subsystem_.voltage();
  }

  status->zeroed = profiled_subsystem_.zeroed();

  profiled_subsystem_.PopulateStatus(status);
  status->estopped = (state_ == State::ESTOP);
  status->state = static_cast<int32_t>(state_);
}

}  // namespace control_loops
}  // namespace frc971