#ifndef Y2018_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_
#define Y2018_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_

#include <memory>

#include "aos/controls/control_loop.h"
#include "aos/events/event_loop.h"
#include "aos/time/time.h"
#include "frc971/control_loops/drivetrain/drivetrain_output_generated.h"
#include "frc971/control_loops/state_feedback_loop.h"
#include "y2018/control_loops/superstructure/arm/arm.h"
#include "y2018/control_loops/superstructure/intake/intake.h"
#include "y2018/control_loops/superstructure/superstructure_goal_generated.h"
#include "y2018/control_loops/superstructure/superstructure_output_generated.h"
#include "y2018/control_loops/superstructure/superstructure_position_generated.h"
#include "y2018/control_loops/superstructure/superstructure_status_generated.h"
#include "y2018/status_light_generated.h"
#include "y2018/vision/vision_generated.h"

namespace y2018 {
namespace control_loops {
namespace superstructure {

class Superstructure
    : public ::aos::controls::ControlLoop<Goal, Position, Status, Output> {
 public:
  explicit Superstructure(::aos::EventLoop *event_loop,
                          const ::std::string &name = "/superstructure");

  const intake::IntakeSide &intake_left() const { return intake_left_; }
  const intake::IntakeSide &intake_right() const { return intake_right_; }
  const arm::Arm &arm() const { return arm_; }

 protected:
  virtual void RunIteration(const Goal *unsafe_goal, const Position *position,
                            aos::Sender<Output>::Builder *output,
                            aos::Sender<Status>::Builder *status) override;

 private:
  // Sends the status light message for the 3 colors provided.
  void SendColors(float red, float green, float blue);

  ::aos::Sender<::y2018::StatusLight> status_light_sender_;
  ::aos::Fetcher<::y2018::vision::VisionStatus> vision_status_fetcher_;
  ::aos::Fetcher<::frc971::control_loops::drivetrain::Output>
      drivetrain_output_fetcher_;

  intake::IntakeSide intake_left_;
  intake::IntakeSide intake_right_;
  arm::Arm arm_;

  // The last centering error. This is the distance that the center of the two
  // intakes is away from 0.
  double last_intake_center_error_ = 0.0;
  // The last distance that the box distance lidar measured.
  double last_box_distance_ = 0.0;
  // State variable for the box velocity low pass filter.
  double filtered_box_velocity_ = 0.0;

  enum class RotationState {
    NOT_ROTATING = 0,
    ROTATING_LEFT = 1,
    ROTATING_RIGHT = 2,
    STUCK = 3
  };

  RotationState rotation_state_ = RotationState::NOT_ROTATING;
  int rotation_count_ = 0;
  int stuck_count_ = 0;
  ::aos::monotonic_clock::time_point last_stuck_time_ =
      ::aos::monotonic_clock::min_time;
  ::aos::monotonic_clock::time_point last_unstuck_time_ =
      ::aos::monotonic_clock::min_time;

  DISALLOW_COPY_AND_ASSIGN(Superstructure);
};

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2018

#endif  // Y2018_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_
