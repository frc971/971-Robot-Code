#ifndef Y2024_CONTROL_LOOPS_SUPERSTRUCTURE_SHOOTER_H_
#define Y2024_CONTROL_LOOPS_SUPERSTRUCTURE_SHOOTER_H_

#include "frc971/control_loops/catapult/catapult.h"
#include "frc971/control_loops/catapult/catapult_goal_static.h"
#include "frc971/control_loops/static_zeroing_single_dof_profiled_subsystem.h"
#include "frc971/shooter_interpolation/interpolation.h"
#include "frc971/zeroing/pot_and_absolute_encoder.h"
#include "y2024/constants.h"
#include "y2024/constants/constants_generated.h"
#include "y2024/control_loops/superstructure/aiming.h"
#include "y2024/control_loops/superstructure/collision_avoidance.h"
#include "y2024/control_loops/superstructure/superstructure_can_position_generated.h"
#include "y2024/control_loops/superstructure/superstructure_goal_generated.h"
#include "y2024/control_loops/superstructure/superstructure_position_generated.h"
#include "y2024/control_loops/superstructure/superstructure_status_generated.h"

namespace y2024::control_loops::superstructure {

class Debouncer {
 public:
  Debouncer(std::chrono::nanoseconds rising_delay,
            std::chrono::nanoseconds falling_delay)
      : rising_delay_(rising_delay), falling_delay_(falling_delay) {}

  void Update(bool state, aos::monotonic_clock::time_point now) {
    if (state_transition_ != state) {
      transition_time_ = now;
      state_transition_ = state;
    }

    if (state != output_state_) {
      if (state) {
        output_state_ = now > transition_time_ + rising_delay_;
      } else {
        output_state_ = !(now > transition_time_ + falling_delay_);
      }
    }
  }

  bool state() const { return output_state_; }

 private:
  const std::chrono::nanoseconds rising_delay_;
  const std::chrono::nanoseconds falling_delay_;

  bool state_transition_ = false;
  bool output_state_ = false;
  aos::monotonic_clock::time_point transition_time_ =
      aos::monotonic_clock::min_time;
};

// The shooter class will control the various subsystems involved in the
// shooter- the turret, altitude, and catapult.
class Shooter {
 public:
  using PotAndAbsoluteEncoderSubsystem =
      ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystem<
          ::frc971::zeroing::PotAndAbsoluteEncoderZeroingEstimator,
          ::frc971::control_loops::PotAndAbsoluteEncoderProfiledJointStatus>;

  using CatapultSubsystem =
      ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystem<
          ::frc971::zeroing::PotAndAbsoluteEncoderZeroingEstimator,
          ::frc971::control_loops::PotAndAbsoluteEncoderProfiledJointStatus,
          ::frc971::zeroing::PotAndAbsoluteEncoderZeroingEstimator,
          aos::util::AsymmetricTrapezoidProfile>;

  Shooter(aos::EventLoop *event_loop, const Constants *robot_constants);

  void Reset() {
    catapult_.Reset();
    turret_.Reset();
    altitude_.Reset();
  }

  void Estop() {
    catapult_.Estop();
    turret_.Estop();
    altitude_.Estop();
  }

  bool zeroed() {
    return catapult_.zeroed() && turret_.zeroed() && altitude_.zeroed();
  }

  bool estopped() {
    return catapult_.estopped() && turret_.estopped() && altitude_.estopped();
  }

  inline const PotAndAbsoluteEncoderSubsystem &turret() const {
    return turret_;
  }

  inline const PotAndAbsoluteEncoderSubsystem &altitude() const {
    return altitude_;
  }

  flatbuffers::Offset<ShooterStatus> Iterate(
      const Position *position, const ShooterGoal *shooter_goal, bool fire,
      double *catapult_output, double *altitude_output, double *turret_output,
      double *retention_roller_output,
      double *retention_roller_stator_current_limit, double battery_voltage,
      /* Hacky way to use collision avoidance in this class */
      CollisionAvoidance *collision_avoidance,
      const double intake_pivot_position, double *max_turret_intake_position,
      double *min_intake_pivot_position,
      /* If true, go to extend collision avoidance position */ bool standby,
      flatbuffers::FlatBufferBuilder *fbb,
      aos::monotonic_clock::time_point monotonic_now);

 private:
  CatapultState state_ = CatapultState::RETRACTING;

  bool CatapultClose() const {
    return (std::abs(catapult_.estimated_position() -
                     catapult_.unprofiled_goal(0, 0)) < 0.05 &&
            std::abs(catapult_.estimated_velocity()) < 0.5);
  }

  aos::Fetcher<frc971::control_loops::drivetrain::Status>
      drivetrain_status_fetcher_;

  aos::Fetcher<y2024::control_loops::superstructure::CANPosition>
      superstructure_can_position_fetcher_;

  const Constants *robot_constants_;

  CatapultSubsystem catapult_;

  PotAndAbsoluteEncoderSubsystem turret_;
  PotAndAbsoluteEncoderSubsystem altitude_;

  Aimer aimer_;

  frc971::shooter_interpolation::InterpolationTable<
      y2024::constants::Values::ShotParams>
      interpolation_table_;

  Debouncer debouncer_;
};

}  // namespace y2024::control_loops::superstructure

#endif
