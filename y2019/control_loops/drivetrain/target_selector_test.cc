#include "y2019/control_loops/drivetrain/target_selector.h"

#include "aos/events/simulated_event_loop.h"
#include "gtest/gtest.h"
#include "y2019/control_loops/superstructure/superstructure_goal_generated.h"

namespace y2019 {
namespace control_loops {
namespace testing {

typedef ::frc971::control_loops::TypedPose<double> Pose;
typedef ::Eigen::Matrix<double, 5, 1> State;

namespace {
// Accessors to get some useful particular targets on the field:
Pose HPSlotLeft() { return constants::Field().targets()[7].pose(); }
Pose CargoNearLeft() { return constants::Field().targets()[2].pose(); }
Pose RocketHatchFarLeft() { return constants::Field().targets()[6].pose(); }
Pose RocketPortal() { return constants::Field().targets()[4].pose(); }
double HatchRadius() { return constants::Field().targets()[6].radius(); }
}  // namespace

// Tests the target selector with:
// -The [x, y, theta, left_vel, right_vel] state to test at
// -The current driver commanded speed.
// -Whether we expect to see a target.
// -If (1) is true, the pose we expect to get back.
struct TestParams {
  State state;
  bool ball_mode;
  drivetrain::SelectionHint selection_hint;
  double command_speed;
  bool expect_target;
  Pose expected_pose;
  double expected_radius;
};
class TargetSelectorParamTest : public ::testing::TestWithParam<TestParams> {
 public:
  TargetSelectorParamTest()
      : configuration_(aos::configuration::ReadConfig("y2019/config.json")),
        event_loop_factory_(&configuration_.message()),
        event_loop_(this->event_loop_factory_.MakeEventLoop("drivetrain")),
        test_event_loop_(this->event_loop_factory_.MakeEventLoop("test")),
        target_selector_hint_sender_(
            test_event_loop_->MakeSender<
                ::y2019::control_loops::drivetrain::TargetSelectorHint>(
                "/drivetrain")),
        superstructure_goal_sender_(
            test_event_loop_
                ->MakeSender<::y2019::control_loops::superstructure::Goal>(
                    "/superstructure")) {}

 private:
  aos::FlatbufferDetachedBuffer<aos::Configuration> configuration_;
  ::aos::SimulatedEventLoopFactory event_loop_factory_;

 protected:
  ::std::unique_ptr<::aos::EventLoop> event_loop_;
  ::std::unique_ptr<::aos::EventLoop> test_event_loop_;

  ::aos::Sender<::y2019::control_loops::drivetrain::TargetSelectorHint>
      target_selector_hint_sender_;
  ::aos::Sender<::y2019::control_loops::superstructure::Goal>
      superstructure_goal_sender_;
};

TEST_P(TargetSelectorParamTest, ExpectReturn) {
  TargetSelector selector(event_loop_.get());
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    superstructure::SuctionGoal::Builder suction_builder =
        builder.MakeBuilder<superstructure::SuctionGoal>();

    suction_builder.add_gamepiece_mode(GetParam().ball_mode ? 0 : 1);

    flatbuffers::Offset<superstructure::SuctionGoal> suction_offset =
        suction_builder.Finish();

    superstructure::Goal::Builder goal_builder =
        builder.MakeBuilder<superstructure::Goal>();

    goal_builder.add_suction(suction_offset);
    ASSERT_TRUE(builder.Send(goal_builder.Finish()));
  }
  {
    auto builder = target_selector_hint_sender_.MakeBuilder();
    ASSERT_TRUE(builder.Send(drivetrain::CreateTargetSelectorHint(
        *builder.fbb(), GetParam().selection_hint)));
  }
  bool expect_target = GetParam().expect_target;
  const State state = GetParam().state;
  ASSERT_EQ(expect_target,
            selector.UpdateSelection(state, GetParam().command_speed))
      << "We expected a return of " << expect_target << " at state "
      << state.transpose();
  if (expect_target) {
    const Pose expected_pose = GetParam().expected_pose;
    const Pose actual_pose = selector.TargetPose();
    const ::Eigen::Vector3d expected_pos = expected_pose.abs_pos();
    const ::Eigen::Vector3d actual_pos = actual_pose.abs_pos();
    const double expected_angle = expected_pose.abs_theta();
    const double actual_angle = actual_pose.abs_theta();
    EXPECT_EQ(expected_pos, actual_pos)
        << "Expected the pose to be at " << expected_pos.transpose()
        << " but got " << actual_pos.transpose() << " with the robot at "
        << state.transpose();
    EXPECT_EQ(expected_angle, actual_angle);
    EXPECT_EQ(GetParam().expected_radius, selector.TargetRadius());
    EXPECT_EQ(expected_angle, actual_angle);
  }
}

INSTANTIATE_TEST_CASE_P(
    TargetSelectorTest, TargetSelectorParamTest,
    ::testing::Values(
        // When we are far away from anything, we should not register any
        // targets:
        TestParams{(State() << 0.0, 0.0, 0.0, 1.0, 1.0).finished(),
                   /*ball_mode=*/false,
                   drivetrain::SelectionHint::NONE,
                   1.0,
                   false,
                   {},
                   /*expected_radius=*/0.0},
        // Aim for a human-player spot; at low speeds we should not register
        // anything.
        TestParams{(State() << 4.0, 2.0, M_PI, 0.05, 0.05).finished(),
                   /*ball_mode=*/false,
                   drivetrain::SelectionHint::NONE,
                   0.05,
                   false,
                   {},
                   /*expected_radius=*/0.0},
        TestParams{(State() << 4.0, 2.0, M_PI, -0.05, -0.05).finished(),
                   /*ball_mode=*/false,
                   drivetrain::SelectionHint::NONE,
                   -0.05,
                   false,
                   {},
                   /*expected_radius=*/0.0},
        TestParams{(State() << 4.0, 2.0, M_PI, 0.5, 0.5).finished(),
                   /*ball_mode=*/false, drivetrain::SelectionHint::NONE, 1.0,
                   true, HPSlotLeft(), /*expected_radius=*/0.0},
        // Put ourselves between the rocket and cargo ship; we should see the
        // hatches driving one direction and the near cargo ship port the other.
        // We also command a speed opposite the current direction of motion and
        // confirm that that behaves as expected.
        TestParams{(State() << 6.0, 2.0, -M_PI_2, -0.5, -0.5).finished(),
                   /*ball_mode=*/false, drivetrain::SelectionHint::NONE, 1.0,
                   true, CargoNearLeft(), /*expected_radius=*/HatchRadius()},
        TestParams{(State() << 6.0, 2.0, M_PI_2, 0.5, 0.5).finished(),
                   /*ball_mode=*/false, drivetrain::SelectionHint::NONE, -1.0,
                   true, CargoNearLeft(), /*expected_radius=*/HatchRadius()},
        TestParams{(State() << 6.0, 2.0, -M_PI_2, 0.5, 0.5).finished(),
                   /*ball_mode=*/false, drivetrain::SelectionHint::NONE, -1.0,
                   true, RocketHatchFarLeft(),
                   /*expected_radius=*/HatchRadius()},
        TestParams{(State() << 6.0, 2.0, M_PI_2, -0.5, -0.5).finished(),
                   /*ball_mode=*/false, drivetrain::SelectionHint::NONE, 1.0,
                   true, RocketHatchFarLeft(),
                   /*expected_radius=*/HatchRadius()},
        // And we shouldn't see anything spinning in place:
        TestParams{(State() << 6.0, 2.0, M_PI_2, -0.5, 0.5).finished(),
                   /*ball_mode=*/false,
                   drivetrain::SelectionHint::NONE,
                   0.0,
                   false,
                   {},
                   /*expected_radius=*/0.0},
        // Drive backwards off the field--we should not see anything.
        TestParams{(State() << -0.1, 0.0, 0.0, -0.5, -0.5).finished(),
                   /*ball_mode=*/false,
                   drivetrain::SelectionHint::NONE,
                   -1.0,
                   false,
                   {},
                   /*expected_radius=*/0.0},
        // In ball mode, we should be able to see the portal, and get zero
        // radius.
        TestParams{(State() << 6.0, 2.0, M_PI_2, 0.5, 0.5).finished(),
                   /*ball_mode=*/true, drivetrain::SelectionHint::NONE, 1.0,
                   true, RocketPortal(),
                   /*expected_radius=*/0.0},
        // Reversing direction should get cargo ship with zero radius.
        TestParams{(State() << 6.0, 2.0, M_PI_2, 0.5, 0.5).finished(),
                   /*ball_mode=*/true, drivetrain::SelectionHint::NONE, -1.0,
                   true, CargoNearLeft(),
                   /*expected_radius=*/0.0}));

}  // namespace testing
}  // namespace control_loops
}  // namespace y2019
