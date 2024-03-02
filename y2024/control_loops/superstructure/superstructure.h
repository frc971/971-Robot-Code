#ifndef Y2024_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_
#define Y2024_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_

#include "aos/events/event_loop.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/time/time.h"
#include "frc971/constants/constants_sender_lib.h"
#include "frc971/control_loops/control_loop.h"
#include "frc971/control_loops/drivetrain/drivetrain_can_position_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_status_generated.h"
#include "frc971/zeroing/absolute_encoder.h"
#include "frc971/zeroing/pot_and_absolute_encoder.h"
#include "y2024/constants.h"
#include "y2024/constants/constants_generated.h"
#include "y2024/control_loops/superstructure/shooter.h"
#include "y2024/control_loops/superstructure/superstructure_goal_generated.h"
#include "y2024/control_loops/superstructure/superstructure_output_generated.h"
#include "y2024/control_loops/superstructure/superstructure_position_generated.h"
#include "y2024/control_loops/superstructure/superstructure_status_generated.h"

namespace y2024::control_loops::superstructure {

class Superstructure
    : public ::frc971::controls::ControlLoop<Goal, Position, Status, Output> {
 public:
  using AbsoluteEncoderSubsystem =
      ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystem<
          ::frc971::zeroing::AbsoluteEncoderZeroingEstimator,
          ::frc971::control_loops::AbsoluteEncoderProfiledJointStatus>;

  using PotAndAbsoluteEncoderSubsystem =
      ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystem<
          ::frc971::zeroing::PotAndAbsoluteEncoderZeroingEstimator,
          ::frc971::control_loops::PotAndAbsoluteEncoderProfiledJointStatus>;

  explicit Superstructure(::aos::EventLoop *event_loop,
                          const ::std::string &name = "/superstructure");

  inline const AbsoluteEncoderSubsystem &intake_pivot() const {
    return intake_pivot_;
  }

  inline const PotAndAbsoluteEncoderSubsystem &climber() const {
    return climber_;
  }

  inline const Shooter &shooter() const { return shooter_; }
  inline const PotAndAbsoluteEncoderSubsystem &extend() const {
    return extend_;
  }

  double robot_velocity() const;

 protected:
  virtual void RunIteration(const Goal *unsafe_goal, const Position *position,
                            aos::Sender<Output>::Builder *output,
                            aos::Sender<Status>::Builder *status) override;

 private:
  frc971::constants::ConstantsFetcher<Constants> constants_fetcher_;
  const Constants *robot_constants_;
  aos::Fetcher<frc971::control_loops::drivetrain::Status>
      drivetrain_status_fetcher_;
  aos::Fetcher<aos::JoystickState> joystick_state_fetcher_;

  CollisionAvoidance collision_avoidance_;

  aos::Alliance alliance_ = aos::Alliance::kInvalid;

  bool catapult_requested_ = false;

  SuperstructureState state_ = SuperstructureState::IDLE;

  NoteGoal requested_note_goal_ = NoteGoal::NONE;

  aos::monotonic_clock::time_point intake_end_time_ =
      aos::monotonic_clock::time_point::min();

  aos::monotonic_clock::time_point loading_catapult_start_time_ =
      aos::monotonic_clock::time_point::min();

  AbsoluteEncoderSubsystem intake_pivot_;
  PotAndAbsoluteEncoderSubsystem climber_;

  Shooter shooter_;

  PotAndAbsoluteEncoderSubsystem extend_;
  DISALLOW_COPY_AND_ASSIGN(Superstructure);
};

}  // namespace y2024::control_loops::superstructure

#endif  // Y2024_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_
