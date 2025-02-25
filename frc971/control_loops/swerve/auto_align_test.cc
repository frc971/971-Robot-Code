#include "frc971/control_loops/swerve/auto_align.h"

#include <math.h>

#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "gtest/gtest.h"

#include "aos/events/event_loop.h"
#include "aos/events/simulated_event_loop.h"
#include "frc971/control_loops/control_loop_test.h"
#include "frc971/control_loops/swerve/position_goal_static.h"

namespace frc971::control_loops::swerve::test {
class AutoAlignSimulator {
 public:
  AutoAlignSimulator(::aos::EventLoop *event_loop)
      : dt_(std::chrono::milliseconds(5)),
        swerve_goal_fetcher_(
            event_loop->MakeFetcher<frc971::control_loops::swerve::Goal>(
                "/autonomous_auto_align")),
        position_goal_sender_(
            event_loop
                ->MakeSender<frc971::control_loops::swerve::PositionGoalStatic>(
                    "/autonomous_auto_align")),
        localizer_state_sender_(
            event_loop
                ->MakeSender<frc971::control_loops::swerve::LocalizerState>(
                    "/localizer")),
        auto_align_(event_loop) {
    phased_loop_handle_ = event_loop->AddPhasedLoop(
        [this](int) {
          SendMessage();
          auto_align_.Iterate();
          Simulate();
        },
        dt_);
  }

  void setPosition(double x, double y, double theta) {
    robot_x_ = x;
    robot_y_ = y;
    robot_theta_ = theta;
  }

  void SendMessage() {
    ::aos::Sender<frc971::control_loops::swerve::LocalizerState>::Builder
        localizer_state = localizer_state_sender_.MakeBuilder();

    LocalizerState::Builder localizer_state_builder =
        localizer_state.MakeBuilder<LocalizerState>();

    localizer_state_builder.add_x(robot_x_);
    localizer_state_builder.add_y(robot_y_);
    localizer_state_builder.add_theta(robot_theta_);

    EXPECT_EQ(localizer_state.Send(localizer_state_builder.Finish()),
              aos::RawSender::Error::kOk);
  }

  void Simulate() {
    CHECK(swerve_goal_fetcher_.Fetch()) << "swerve_goal_fetcher_ is nullptr!";
    const frc971::control_loops::swerve::JoystickGoal *joystick_goal =
        swerve_goal_fetcher_.get()->joystick_goal();
    std::chrono::duration<double, std::ratio<1, 1>> dt_sec = dt_;
    robot_x_ += joystick_goal->vx() * dt_sec.count();
    robot_y_ += joystick_goal->vy() * dt_sec.count();
    robot_theta_ += joystick_goal->omega() * dt_sec.count();
    LOG(INFO) << "vx: " << joystick_goal->vx() << "vy: " << joystick_goal->vy()
              << "omega: " << joystick_goal->omega();
  }

  void setGoal(double x, double y, double theta) {
    auto position_goal_builder = position_goal_sender_.MakeStaticBuilder();

    position_goal_builder->set_x(x);
    position_goal_builder->set_y(y);
    position_goal_builder->set_theta(theta);
    position_goal_builder.CheckOk(position_goal_builder.Send());
  }

  double robot_x() { return robot_x_; }

  double robot_y() { return robot_y_; }

  double robot_theta() { return robot_theta_; }

 private:
  double robot_x_ = 0.0;
  double robot_y_ = 0.0;
  double robot_theta_ = 0.0;
  const std::chrono::nanoseconds dt_;
  ::aos::PhasedLoopHandler *phased_loop_handle_ = nullptr;

  ::aos::Fetcher<frc971::control_loops::swerve::Goal> swerve_goal_fetcher_;
  ::aos::Sender<frc971::control_loops::swerve::PositionGoalStatic>
      position_goal_sender_;
  aos::Sender<frc971::control_loops::swerve::LocalizerState>
      localizer_state_sender_;
  AutoAlign auto_align_;
};
}  // namespace frc971::control_loops::swerve::test

class AutoAlignTest : public frc971::testing::ControlLoopTest {
 public:
  AutoAlignTest()
      : frc971::testing::ControlLoopTest(aos::configuration::ReadConfig(
            "frc971/control_loops/swerve/aos_config.json")),
        event_loop_(MakeEventLoop("test", nullptr)),
        simulator_(event_loop_.get()) {}

  void setGoal(double x, double y, double theta) {
    goal_x_ = x;
    goal_y_ = y;
    goal_theta_ = theta;
    simulator_.setGoal(x, y, theta);
  }

  void VerifyNearGoal() {
    constexpr double kEps = 0.005;
    EXPECT_NEAR(simulator_.robot_x(), goal_x_, kEps);
    EXPECT_NEAR(simulator_.robot_y(), goal_y_, kEps);
    EXPECT_NEAR(simulator_.robot_theta(), goal_theta_, kEps);
    LOG(INFO) << "x: " << simulator_.robot_x() << "y: " << simulator_.robot_y()
              << "theta: " << simulator_.robot_theta();
  }

 private:
  ::std::unique_ptr<::aos::EventLoop> event_loop_;
  frc971::control_loops::swerve::test::AutoAlignSimulator simulator_;
  double goal_x_ = 0.0;
  double goal_y_ = 0.0;
  double goal_theta_ = 0.0;
};

TEST_F(AutoAlignTest, AlignToDesiredState) {
  setGoal(15.0, -5.0, M_PI / 2);
  RunFor(std::chrono::seconds(15));
  VerifyNearGoal();
}
