#ifndef Y2023_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_
#define Y2023_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_

#include "aos/events/event_loop.h"
#include "aos/json_to_flatbuffer.h"
#include "frc971/constants/constants_sender_lib.h"
#include "frc971/control_loops/control_loop.h"
#include "frc971/control_loops/drivetrain/drivetrain_can_position_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_status_generated.h"
#include "y2023/constants.h"
#include "y2023/constants/constants_generated.h"
#include "y2023/control_loops/superstructure/arm/arm.h"
#include "y2023/control_loops/superstructure/arm/arm_trajectories_generated.h"
#include "y2023/control_loops/superstructure/end_effector.h"
#include "y2023/control_loops/superstructure/superstructure_goal_generated.h"
#include "y2023/control_loops/superstructure/superstructure_output_generated.h"
#include "y2023/control_loops/superstructure/superstructure_position_generated.h"
#include "y2023/control_loops/superstructure/superstructure_status_generated.h"

using y2023::control_loops::superstructure::arm::ArmTrajectories;
using y2023::control_loops::superstructure::arm::TrajectoryAndParams;

namespace y2023 {
namespace control_loops {
namespace superstructure {

class Superstructure
    : public ::frc971::controls::ControlLoop<Goal, Position, Status, Output> {
 public:
  using RelativeEncoderSubsystem =
      ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystem<
          ::frc971::zeroing::RelativeEncoderZeroingEstimator,
          ::frc971::control_loops::RelativeEncoderProfiledJointStatus>;

  using PotAndAbsoluteEncoderSubsystem =
      ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystem<
          ::frc971::zeroing::PotAndAbsoluteEncoderZeroingEstimator,
          ::frc971::control_loops::PotAndAbsoluteEncoderProfiledJointStatus>;

  using AbsoluteEncoderSubsystem =
      ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystem<
          ::frc971::zeroing::AbsoluteEncoderZeroingEstimator,
          ::frc971::control_loops::AbsoluteEncoderProfiledJointStatus>;

  explicit Superstructure(
      ::aos::EventLoop *event_loop,
      std::shared_ptr<const constants::Values> values,
      const aos::FlatbufferVector<ArmTrajectories> &arm_trajectories,
      const ::std::string &name = "/superstructure");

  double robot_velocity() const;

  inline const arm::Arm &arm() const { return arm_; }
  inline const EndEffector &end_effector() const { return end_effector_; }
  inline const AbsoluteEncoderSubsystem &wrist() const { return wrist_; }

  static const aos::FlatbufferVector<ArmTrajectories> GetArmTrajectories(
      const std::string &filename) {
    return aos::FileToFlatbuffer<arm::ArmTrajectories>(filename);
  }

 protected:
  virtual void RunIteration(const Goal *unsafe_goal, const Position *position,
                            aos::Sender<Output>::Builder *output,
                            aos::Sender<Status>::Builder *status) override;

 private:
  // Returns the Y coordinate of a game piece given the time-of-flight reading.
  std::optional<double> LateralOffsetForTimeOfFlight(double reading);

  std::shared_ptr<const constants::Values> values_;
  frc971::constants::ConstantsFetcher<Constants> constants_fetcher_;

  aos::Fetcher<frc971::control_loops::drivetrain::Status>
      drivetrain_status_fetcher_;
  aos::Fetcher<aos::JoystickState> joystick_state_fetcher_;

  arm::Arm arm_;
  EndEffector end_effector_;
  AbsoluteEncoderSubsystem wrist_;

  aos::Alliance alliance_ = aos::Alliance::kInvalid;

  DISALLOW_COPY_AND_ASSIGN(Superstructure);
};

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2023

#endif  // Y2023_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_
