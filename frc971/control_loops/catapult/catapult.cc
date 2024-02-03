#include "frc971/control_loops/catapult/catapult.h"

namespace frc971::control_loops::catapult {

const flatbuffers::Offset<
    frc971::control_loops::PotAndAbsoluteEncoderProfiledJointStatus>
Catapult::Iterate(const CatapultGoal *catapult_goal,
                  const PotAndAbsolutePosition *position,
                  double battery_voltage, double *catapult_voltage, bool fire,
                  flatbuffers::FlatBufferBuilder *fbb) {
  const frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoal
      *return_goal =
          catapult_goal != nullptr && catapult_goal->has_return_position()
              ? catapult_goal->return_position()
              : nullptr;

  const bool catapult_disabled =
      catapult_.Correct(return_goal, position, catapult_voltage == nullptr);

  if (catapult_disabled) {
    catapult_state_ = CatapultState::PROFILE;
  } else if (catapult_.running() && catapult_goal != nullptr && fire &&
             !last_firing_) {
    catapult_state_ = CatapultState::FIRING;
    latched_shot_position = catapult_goal->shot_position();
    latched_shot_velocity = catapult_goal->shot_velocity();
  }

  // Don't update last_firing_ if the catapult is disabled, so that we actually
  // end up firing once it's enabled
  if (catapult_.running() && !catapult_disabled) {
    last_firing_ = fire;
  }

  use_profile_ = true;

  switch (catapult_state_) {
    case CatapultState::FIRING: {
      // Select the ball controller.  We should only be firing if we have a
      // ball, or at least should only care about the shot accuracy.
      catapult_.set_controller_index(0);
      // Ok, so we've now corrected.  Next step is to run the MPC.
      //
      // Since there is a unit delay between when we ask for a U and the
      // hardware applies it, we need to run the optimizer for the position at
      // the *next* control loop cycle.

      Eigen::Vector3d next_X = catapult_.estimated_state();
      for (int i = catapult_.controller().plant().coefficients().delayed_u;
           i > 1; --i) {
        next_X = catapult_.controller().plant().A() * next_X +
                 catapult_.controller().plant().B() *
                     catapult_.controller().observer().last_U(i - 1);
      }

      catapult_mpc_.SetState(
          next_X.block<2, 1>(0, 0),
          Eigen::Vector2d(latched_shot_position, latched_shot_velocity));

      const bool solved = catapult_mpc_.Solve();
      current_horizon_ = catapult_mpc_.current_horizon();
      const bool started = catapult_mpc_.started();
      if (solved || started) {
        std::optional<double> solution = catapult_mpc_.Next();

        if (!solution.has_value()) {
          CHECK_NOTNULL(catapult_voltage);
          *catapult_voltage = 0.0;
          if (catapult_mpc_.started()) {
            ++shot_count_;
            // Finished the catapult, time to fire.
            catapult_state_ = CatapultState::RESETTING;
          }
        } else {
          // TODO(austin): Voltage error?
          CHECK_NOTNULL(catapult_voltage);
          if (current_horizon_ == 1) {
            battery_voltage = 12.0;
          }
          *catapult_voltage = std::max(
              0.0, std::min(12.0, (*solution - 0.0 * next_X(2, 0)) * 12.0 /
                                      std::max(battery_voltage, 8.0)));
          use_profile_ = false;
        }
      } else {
        if (!fire) {
          // Eh, didn't manage to solve before it was time to fire.  Give up.
          catapult_state_ = CatapultState::PROFILE;
        }
      }

      if (!use_profile_) {
        catapult_.ForceGoal(catapult_.estimated_position(),
                            catapult_.estimated_velocity());
      }
    }
      if (catapult_state_ != CatapultState::RESETTING) {
        break;
      } else {
        [[fallthrough]];
      }

    case CatapultState::RESETTING:
      if (catapult_.controller().R(1, 0) > 7.0) {
        catapult_.AdjustProfile(7.0, 2000.0);
      } else if (catapult_.controller().R(1, 0) > 0.0) {
        catapult_.AdjustProfile(7.0, 1000.0);
      } else {
        catapult_state_ = CatapultState::PROFILE;
      }
      [[fallthrough]];

    case CatapultState::PROFILE:
      break;
  }

  if (use_profile_) {
    if (catapult_state_ != CatapultState::FIRING) {
      catapult_mpc_.Reset();
    }
    // Select the controller designed for when we have no ball.
    catapult_.set_controller_index(1);

    current_horizon_ = 0u;
    const double output_voltage = catapult_.UpdateController(catapult_disabled);
    if (catapult_voltage != nullptr) {
      *catapult_voltage = output_voltage;
    }
  }

  catapult_.UpdateObserver(catapult_voltage != nullptr
                               ? (*catapult_voltage * battery_voltage / 12.0)
                               : 0.0);

  return catapult_.MakeStatus(fbb);
}

}  // namespace frc971::control_loops::catapult