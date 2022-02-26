#ifndef Y2022_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_
#define Y2022_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_

#include "aos/events/event_loop.h"
#include "frc971/control_loops/control_loop.h"
#include "frc971/control_loops/drivetrain/drivetrain_status_generated.h"
#include "y2022/constants.h"
#include "y2022/control_loops/superstructure/catapult/catapult.h"
#include "y2022/control_loops/superstructure/collision_avoidance.h"
#include "y2022/control_loops/superstructure/superstructure_goal_generated.h"
#include "y2022/control_loops/superstructure/superstructure_output_generated.h"
#include "y2022/control_loops/superstructure/superstructure_position_generated.h"
#include "y2022/control_loops/superstructure/superstructure_status_generated.h"

namespace y2022 {
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

  explicit Superstructure(::aos::EventLoop *event_loop,
                          std::shared_ptr<const constants::Values> values,
                          const ::std::string &name = "/superstructure");

  inline const PotAndAbsoluteEncoderSubsystem &intake_front() const {
    return intake_front_;
  }
  inline const PotAndAbsoluteEncoderSubsystem &intake_back() const {
    return intake_back_;
  }
  inline const PotAndAbsoluteEncoderSubsystem &turret() const {
    return turret_;
  }
  inline const RelativeEncoderSubsystem &climber() const { return climber_; }

  double robot_velocity() const;

 protected:
  virtual void RunIteration(const Goal *unsafe_goal, const Position *position,
                            aos::Sender<Output>::Builder *output,
                            aos::Sender<Status>::Builder *status) override;

 private:
  std::shared_ptr<const constants::Values> values_;

  RelativeEncoderSubsystem climber_;
  PotAndAbsoluteEncoderSubsystem intake_front_;
  PotAndAbsoluteEncoderSubsystem intake_back_;
  PotAndAbsoluteEncoderSubsystem turret_;

  CollisionAvoidance collision_avoidance_;

  aos::Fetcher<frc971::control_loops::drivetrain::Status>
      drivetrain_status_fetcher_;

  DISALLOW_COPY_AND_ASSIGN(Superstructure);

  catapult::Catapult catapult_;
};

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2022

#endif  // Y2022_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_
