#ifndef Y2022_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_
#define Y2022_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_

#include "aos/events/event_loop.h"
#include "frc971/control_loops/control_loop.h"
#include "frc971/control_loops/drivetrain/drivetrain_status_generated.h"
#include "y2022/constants.h"
#include "y2022/control_loops/superstructure/catapult/catapult.h"
#include "y2022/control_loops/superstructure/collision_avoidance.h"
#include "y2022/control_loops/superstructure/superstructure_can_position_generated.h"
#include "y2022/control_loops/superstructure/superstructure_goal_generated.h"
#include "y2022/control_loops/superstructure/superstructure_output_generated.h"
#include "y2022/control_loops/superstructure/superstructure_position_generated.h"
#include "y2022/control_loops/superstructure/superstructure_status_generated.h"
#include "y2022/control_loops/superstructure/turret/aiming.h"
#include "y2022/vision/ball_color_generated.h"

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

  static constexpr double kTurretGoalThreshold = 0.05;
  static constexpr double kTurretGoalLoadingThreshold = 0.70;
  static constexpr double kCatapultGoalThreshold = 0.05;
  // potentiometer will be more noisy
  static constexpr double kFlipperGoalThreshold = 0.05;
  static constexpr double kDiscardingPosition = 0.35;
  static constexpr double kDiscardingVelocity = 6.0;

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
  catapult::Catapult catapult_;

  CollisionAvoidance collision_avoidance_;

  aos::Fetcher<frc971::control_loops::drivetrain::Status>
      drivetrain_status_fetcher_;
  aos::Fetcher<CANPosition> can_position_fetcher_;
  aos::Fetcher<aos::JoystickState> joystick_state_fetcher_;
  aos::Fetcher<y2022::vision::BallColor> ball_color_fetcher_;

  int prev_shot_count_ = 0;

  turret::Aimer aimer_;

  bool flippers_open_ = false;
  bool reseating_in_catapult_ = false;
  bool fire_ = false;
  bool discarding_ball_ = false;
  aos::Alliance alliance_ = aos::Alliance::kInvalid;
  aos::Alliance ball_color_ = aos::Alliance::kInvalid;

  aos::monotonic_clock::time_point front_intake_beambreak_timer_ =
      aos::monotonic_clock::min_time;
  aos::monotonic_clock::time_point back_intake_beambreak_timer_ =
      aos::monotonic_clock::min_time;
  aos::monotonic_clock::time_point transferring_timer_ =
      aos::monotonic_clock::min_time;
  aos::monotonic_clock::time_point loading_timer_ =
      aos::monotonic_clock::min_time;
  aos::monotonic_clock::time_point flipper_opening_start_time_ =
      aos::monotonic_clock::min_time;
  SuperstructureState state_ = SuperstructureState::IDLE;
  bool front_intake_has_ball_ = false;
  bool back_intake_has_ball_ = false;
  std::optional<double> last_shot_angle_ = std::nullopt;
  RequestedIntake turret_intake_state_ = RequestedIntake::kFront;

  DISALLOW_COPY_AND_ASSIGN(Superstructure);
};

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2022

#endif  // Y2022_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_
