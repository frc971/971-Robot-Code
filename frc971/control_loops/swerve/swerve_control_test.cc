#include <unistd.h>

#include <chrono>
#include <memory>
#include <vector>

#include "gtest/gtest.h"

#include "aos/events/shm_event_loop.h"
#include "frc971/control_loops/control_loop_test.h"
#include "frc971/control_loops/swerve/swerve_control_loops.h"
#include "frc971/control_loops/swerve/swerve_drivetrain_can_position_generated.h"
#include "frc971/control_loops/swerve/swerve_drivetrain_goal_generated.h"
#include "frc971/control_loops/swerve/swerve_drivetrain_output_generated.h"
#include "frc971/control_loops/swerve/swerve_drivetrain_position_generated.h"
#include "frc971/control_loops/swerve/swerve_drivetrain_status_generated.h"
#include "frc971/control_loops/team_number_test_environment.h"

using namespace std;

namespace frc971::control_loops::swerve::testing {
namespace chrono = ::std::chrono;
using aos::monotonic_clock;

// Class which simulates stuff and sends out queue messages with the position.
class SwerveControlSimulation {
 public:
  SwerveControlSimulation(::aos::EventLoop *event_loop, chrono::nanoseconds dt)
      : event_loop_(event_loop),
        position_sender_(event_loop_->MakeSender<Position>("/swerve")),
        can_position_sender_(event_loop_->MakeSender<CanPosition>("/swerve")),
        goal_fetcher_(event_loop_->MakeFetcher<Goal>("/swerve")),
        status_fetcher_(event_loop_->MakeFetcher<Status>("/swerve")),
        output_fetcher_(event_loop_->MakeFetcher<Output>("/swerve")) {
    event_loop_->AddPhasedLoop(
        [this](int) {
          if (!first_) {
            Simulate();
          }
          first_ = false;
          SendPositionMessage();
        },
        dt);
  }

  // Sends a queue message with the position data.
  void SendPositionMessage() {
    auto builder = position_sender_.MakeBuilder();

    Position::Builder position_builder = builder.MakeBuilder<Position>();

    EXPECT_EQ(builder.Send(position_builder.Finish()),
              aos::RawSender::Error::kOk);
  }

  void VerifyNearGoal() {
    goal_fetcher_.Fetch();
    status_fetcher_.Fetch();

    ASSERT_TRUE(goal_fetcher_.get() != nullptr) << ": No goal";
    ASSERT_TRUE(status_fetcher_.get() != nullptr) << ": No status";

    constexpr double kEpsRotationAngle = 0.03;
    constexpr double kEpsRotationVelocity = 0.03;
    constexpr double kEpsTranslationSpeed = 0.03;

    std::vector<const SwerveModuleStatus *> modules_status{
        status_fetcher_->front_left_status(),
        status_fetcher_->front_right_status(),
        status_fetcher_->back_left_status(),
        status_fetcher_->back_right_status()};

    for (auto &module_status : modules_status) {
      EXPECT_NEAR(module_status->rotation()->position(),
                  module_status->rotation()->goal_position(),
                  kEpsRotationAngle);
      EXPECT_NEAR(module_status->rotation()->velocity(),
                  module_status->rotation()->goal_velocity(),
                  kEpsRotationVelocity);
      EXPECT_NEAR(module_status->goal_translation_speed(),
                  module_status->translation_speed(), kEpsTranslationSpeed);
    }
  }

  // Simulates basic control loop for a single timestep.
  void Simulate() { EXPECT_TRUE(output_fetcher_.Fetch()); }

 private:
  ::aos::EventLoop *event_loop_;

  ::aos::Sender<Position> position_sender_;
  ::aos::Sender<CanPosition> can_position_sender_;
  ::aos::Sender<Goal> goal_sender_;

  ::aos::Fetcher<CanPosition> can_position_fetcher_;
  ::aos::Fetcher<Position> position_fetcher_;
  ::aos::Fetcher<Goal> goal_fetcher_;
  ::aos::Fetcher<Status> status_fetcher_;
  ::aos::Fetcher<Output> output_fetcher_;

  bool first_ = true;
};

class SwerveControlLoopTest : public ::frc971::testing::ControlLoopTest {
 public:
  SwerveControlLoopTest()
      : ::frc971::testing::ControlLoopTest(
            aos::configuration::ReadConfig(
                "frc971/control_loops/swerve/aos_config.json"),
            chrono::microseconds(5050)),
        swerve_test_event_loop_(MakeEventLoop("test")),
        goal_sender_(swerve_test_event_loop_->MakeSender<Goal>("/swerve")),

        swerve_control_event_loop_(MakeEventLoop("swerve_control")),
        swerve_control_loops_(swerve_control_event_loop_.get(), "/swerve"),

        swerve_control_simulation_event_loop_(MakeEventLoop("simulation")),
        swerve_control_simulation_(swerve_control_simulation_event_loop_.get(),
                                   dt()) {
    set_team_id(control_loops::testing::kTeamNumber);
    SetEnabled(true);
  }

  ::std::unique_ptr<::aos::EventLoop> swerve_test_event_loop_;
  ::aos::Sender<Goal> goal_sender_;

  ::std::unique_ptr<::aos::EventLoop> swerve_control_event_loop_;
  SwerveControlLoops swerve_control_loops_;

  ::std::unique_ptr<::aos::EventLoop> swerve_control_simulation_event_loop_;
  SwerveControlSimulation swerve_control_simulation_;
};

// Tests that the swerve modules' speeds are all set to 0.
TEST_F(SwerveControlLoopTest, SwerveModulesDontMove) {
  {
    auto builder = goal_sender_.MakeBuilder();
    SwerveModuleGoal::Builder swerve_module_builder =
        builder.MakeBuilder<SwerveModuleGoal>();
    swerve_module_builder.add_translation_control_type_goal(
        TranslationControlTypeGoal::CURRENT);
    swerve_module_builder.add_rotation_angle(0.0);
    swerve_module_builder.add_translation_current(0.0);
    swerve_module_builder.add_translation_speed(0.0);
    auto empty_module_offset = swerve_module_builder.Finish();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_front_left_goal(empty_module_offset);
    goal_builder.add_front_right_goal(empty_module_offset);
    goal_builder.add_back_left_goal(empty_module_offset);
    goal_builder.add_back_right_goal(empty_module_offset);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  RunFor(dt() * 2);

  swerve_control_simulation_.VerifyNearGoal();
}

}  // namespace frc971::control_loops::swerve::testing