#ifndef FRC971_CONTROL_LOOPS_CATAPULT_CATAPULT_H_
#define FRC971_CONTROL_LOOPS_CATAPULT_CATAPULT_H_

#include "frc971/control_loops/catapult/catapult_controller.h"
#include "frc971/control_loops/catapult/catapult_goal_generated.h"
#include "frc971/control_loops/static_zeroing_single_dof_profiled_subsystem.h"
#include "frc971/zeroing/pot_and_absolute_encoder.h"

namespace frc971 {
namespace control_loops {
namespace catapult {

// Class to handle transitioning between both the profiled subsystem and the MPC
// for shooting.
class Catapult {
 public:
  Catapult(frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemParams<
               zeroing::PotAndAbsoluteEncoderZeroingEstimator>
               catapult_params,
           StateFeedbackPlant<2, 1, 1> plant)
      : catapult_(catapult_params), catapult_mpc_(std::move(plant), 30) {}

  using PotAndAbsoluteEncoderSubsystem =
      ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystem<
          ::frc971::zeroing::PotAndAbsoluteEncoderZeroingEstimator,
          ::frc971::control_loops::PotAndAbsoluteEncoderProfiledJointStatus>;

  // Resets all state for when WPILib restarts.
  void Reset() { catapult_.Reset(); }

  void Estop() { catapult_.Estop(); }

  bool zeroed() const { return catapult_.zeroed(); }
  bool estopped() const { return catapult_.estopped(); }
  double solve_time() const { return catapult_mpc_.solve_time(); }

  uint8_t mpc_horizon() const { return current_horizon_; }

  bool mpc_active() const { return !use_profile_; }

  // Returns the number of shots taken.
  int shot_count() const { return shot_count_; }

  // Returns the estimated position
  double estimated_position() const { return catapult_.estimated_position(); }

  // Runs either the MPC or the profiled subsystem depending on if we are
  // shooting or not.  Returns the status.
  const flatbuffers::Offset<
      frc971::control_loops::PotAndAbsoluteEncoderProfiledJointStatus>
  Iterate(const CatapultGoal *catapult_goal,
          const PotAndAbsolutePosition *position, double battery_voltage,
          double *catapult_voltage, bool fire,
          flatbuffers::FlatBufferBuilder *fbb);

 private:
  PotAndAbsoluteEncoderSubsystem catapult_;

  frc971::control_loops::catapult::CatapultController catapult_mpc_;

  enum CatapultState { PROFILE, FIRING, RESETTING };

  CatapultState catapult_state_ = CatapultState::PROFILE;

  double latched_shot_position = 0.0;
  double latched_shot_velocity = 0.0;

  bool last_firing_ = false;
  bool use_profile_ = true;

  int shot_count_ = 0;
  uint8_t current_horizon_ = 0u;
};

}  // namespace catapult
}  // namespace control_loops
}  // namespace frc971

#endif  // Y2022_CONTROL_LOOPS_SUPERSTRUCTURE_CATAPULT_CATAPULT_H_
