#ifndef Y2025_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_
#define Y2025_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_

#include "aos/events/event_loop.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/time/time.h"
#include "frc971/constants/constants_sender_lib.h"
#include "frc971/control_loops/control_loop.h"
#include "frc971/zeroing/absolute_encoder.h"
#include "frc971/zeroing/pot_and_absolute_encoder.h"
#include "y2025/constants.h"
#include "y2025/constants/constants_generated.h"
#include "y2025/control_loops/superstructure/superstructure_can_position_generated.h"
#include "y2025/control_loops/superstructure/superstructure_goal_generated.h"
#include "y2025/control_loops/superstructure/superstructure_output_generated.h"
#include "y2025/control_loops/superstructure/superstructure_position_generated.h"
#include "y2025/control_loops/superstructure/superstructure_status_generated.h"

namespace y2025::control_loops::superstructure {

// TODO add real value
static constexpr double kEndEffectorMotorTorqueThreshold = 100;

class Superstructure
    : public ::frc971::controls::ControlLoop<Goal, Position, Status, Output> {
 public:
  using PotAndAbsoluteEncoderSubsystem =
      ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystem<
          ::frc971::zeroing::PotAndAbsoluteEncoderZeroingEstimator,
          ::frc971::control_loops::PotAndAbsoluteEncoderProfiledJointStatus>;
  using AbsoluteEncoderSubsystem =
      ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystem<
          ::frc971::zeroing::AbsoluteEncoderZeroingEstimator,
          ::frc971::control_loops::AbsoluteEncoderProfiledJointStatus>;

  explicit Superstructure(::aos::EventLoop *event_loop,
                          const ::std::string &name = "/superstructure");

  bool GetIntakeComplete();

  const PotAndAbsoluteEncoderSubsystem &elevator() const { return elevator_; }
  const PotAndAbsoluteEncoderSubsystem &pivot() const { return pivot_; }
  const AbsoluteEncoderSubsystem &wrist() const { return wrist_; }

 protected:
  virtual void RunIteration(const Goal *goal, const Position *position,
                            aos::Sender<Output>::Builder *output,
                            aos::Sender<Status>::Builder *status) override;

 private:
  frc971::constants::ConstantsFetcher<Constants> constants_fetcher_;
  const Constants *robot_constants_;
  aos::Fetcher<aos::JoystickState> joystick_state_fetcher_;
  aos::Fetcher<y2025::control_loops::superstructure::Goal>
      auto_superstructure_goal_fetcher_;
  PotAndAbsoluteEncoderSubsystem elevator_;
  PotAndAbsoluteEncoderSubsystem pivot_;
  AbsoluteEncoderSubsystem wrist_;
  aos::Fetcher<CANPosition> rio_can_position_fetcher_;

  aos::Alliance alliance_ = aos::Alliance::kInvalid;
  bool intake_complete_ = false;

  DISALLOW_COPY_AND_ASSIGN(Superstructure);
};

}  // namespace y2025::control_loops::superstructure

#endif  // y2025_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_
