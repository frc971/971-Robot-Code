#include "y2016/control_loops/shooter/shooter.h"

#include <unistd.h>

#include <chrono>
#include <memory>

#include "aos/controls/control_loop_test.h"
#include "frc971/control_loops/team_number_test_environment.h"
#include "gtest/gtest.h"
#include "y2016/control_loops/shooter/shooter.h"
#include "y2016/control_loops/shooter/shooter_goal_generated.h"
#include "y2016/control_loops/shooter/shooter_output_generated.h"
#include "y2016/control_loops/shooter/shooter_plant.h"
#include "y2016/control_loops/shooter/shooter_position_generated.h"
#include "y2016/control_loops/shooter/shooter_status_generated.h"

using ::frc971::control_loops::testing::kTeamNumber;

namespace y2016 {
namespace control_loops {
namespace shooter {
namespace testing {

namespace chrono = ::std::chrono;
using ::aos::monotonic_clock;

class ShooterPlant : public StateFeedbackPlant<2, 1, 1> {
 public:
  explicit ShooterPlant(StateFeedbackPlant<2, 1, 1> &&other)
      : StateFeedbackPlant<2, 1, 1>(::std::move(other)) {}

  void CheckU(const ::Eigen::Matrix<double, 1, 1> &U) override {
    EXPECT_LE(U(0, 0), U_max(0, 0) + 0.00001 + voltage_offset_);
    EXPECT_GE(U(0, 0), U_min(0, 0) - 0.00001 + voltage_offset_);
  }

  double voltage_offset() const { return voltage_offset_; }
  void set_voltage_offset(double voltage_offset) {
    voltage_offset_ = voltage_offset;
  }

 private:
  double voltage_offset_ = 0.0;
};


// Class which simulates the shooter and sends out queue messages with the
// position.
class ShooterSimulation {
 public:
  // Constructs a shooter simulation.
  ShooterSimulation(::aos::EventLoop *event_loop, chrono::nanoseconds dt)
      : event_loop_(event_loop),
        shooter_position_sender_(event_loop_->MakeSender<Position>("/shooter")),
        shooter_output_fetcher_(event_loop_->MakeFetcher<Output>("/shooter")),
        shooter_plant_left_(new ShooterPlant(
            ::y2016::control_loops::shooter::MakeShooterPlant())),
        shooter_plant_right_(new ShooterPlant(
            ::y2016::control_loops::shooter::MakeShooterPlant())) {
    event_loop_->AddPhasedLoop(
        [this](int) {
          // Skip this the first time.
          if (!first_) {
            Simulate();
          }
          first_ = false;
          SendPositionMessage();
        },
        dt);
  }

  // Sends a queue message with the position of the shooter.
  void SendPositionMessage() {
    ::aos::Sender<Position>::Builder builder =
        shooter_position_sender_.MakeBuilder();

    Position::Builder position_builder = builder.MakeBuilder<Position>();

    position_builder.add_theta_left(shooter_plant_left_->Y(0, 0));
    position_builder.add_theta_right(shooter_plant_right_->Y(0, 0));
    builder.Send(position_builder.Finish());
  }

  void set_left_voltage_offset(double voltage_offset) {
    shooter_plant_left_->set_voltage_offset(voltage_offset);
  }

  void set_right_voltage_offset(double voltage_offset) {
    shooter_plant_right_->set_voltage_offset(voltage_offset);
  }

  // Simulates shooter for a single timestep.
  void Simulate() {
    EXPECT_TRUE(shooter_output_fetcher_.Fetch());

    ::Eigen::Matrix<double, 1, 1> U_left;
    ::Eigen::Matrix<double, 1, 1> U_right;
    U_left(0, 0) = shooter_output_fetcher_->voltage_left() +
                   shooter_plant_left_->voltage_offset();
    U_right(0, 0) = shooter_output_fetcher_->voltage_right() +
                    shooter_plant_right_->voltage_offset();

    shooter_plant_left_->Update(U_left);
    shooter_plant_right_->Update(U_right);
  }

 private:
  ::aos::EventLoop *event_loop_;

  ::aos::Sender<Position> shooter_position_sender_;
  ::aos::Fetcher<Output> shooter_output_fetcher_;

  ::std::unique_ptr<ShooterPlant> shooter_plant_left_, shooter_plant_right_;

  bool first_ = true;
};

class ShooterTest : public ::aos::testing::ControlLoopTest {
 protected:
  ShooterTest()
      : ::aos::testing::ControlLoopTest(
            aos::configuration::ReadConfig("y2016/config.json"),
            chrono::microseconds(5000)),
        test_event_loop_(MakeEventLoop("test")),
        shooter_goal_fetcher_(test_event_loop_->MakeFetcher<Goal>("/shooter")),
        shooter_goal_sender_(test_event_loop_->MakeSender<Goal>("/shooter")),
        shooter_status_fetcher_(
            test_event_loop_->MakeFetcher<Status>("/shooter")),
        shooter_output_fetcher_(
            test_event_loop_->MakeFetcher<Output>("/shooter")),
        shooter_event_loop_(MakeEventLoop("shooter")),
        shooter_(shooter_event_loop_.get()),
        shooter_plant_event_loop_(MakeEventLoop("plant")),
        shooter_plant_(shooter_plant_event_loop_.get(), dt()) {
    set_team_id(kTeamNumber);
  }

  void VerifyNearGoal() {
    shooter_goal_fetcher_.Fetch();
    shooter_status_fetcher_.Fetch();

    EXPECT_TRUE(shooter_goal_fetcher_.get() != nullptr);
    EXPECT_TRUE(shooter_status_fetcher_.get() != nullptr);

    EXPECT_NEAR(shooter_goal_fetcher_->angular_velocity(),
                shooter_status_fetcher_->left()->angular_velocity(), 10.0);
    EXPECT_NEAR(shooter_goal_fetcher_->angular_velocity(),
                shooter_status_fetcher_->right()->angular_velocity(), 10.0);

    EXPECT_NEAR(shooter_goal_fetcher_->angular_velocity(),
                shooter_status_fetcher_->left()->avg_angular_velocity(), 10.0);
    EXPECT_NEAR(shooter_goal_fetcher_->angular_velocity(),
                shooter_status_fetcher_->right()->avg_angular_velocity(), 10.0);
  }

  ::std::unique_ptr<::aos::EventLoop> test_event_loop_;
  ::aos::Fetcher<Goal> shooter_goal_fetcher_;
  ::aos::Sender<Goal> shooter_goal_sender_;
  ::aos::Fetcher<Status> shooter_status_fetcher_;
  ::aos::Fetcher<Output> shooter_output_fetcher_;

  // Create a control loop and simulation.
  ::std::unique_ptr<::aos::EventLoop> shooter_event_loop_;
  Shooter shooter_;

  ::std::unique_ptr<::aos::EventLoop> shooter_plant_event_loop_;
  ShooterSimulation shooter_plant_;
};

// Tests that the shooter does nothing when the goal is zero.
TEST_F(ShooterTest, DoesNothing) {
  SetEnabled(true);
  {
    auto builder = shooter_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_angular_velocity(0.0);
    EXPECT_TRUE(builder.Send(goal_builder.Finish()));
  }

  RunFor(dt() * 3 / 2);

  VerifyNearGoal();

  EXPECT_TRUE(shooter_output_fetcher_.Fetch());
  EXPECT_EQ(shooter_output_fetcher_->voltage_left(), 0.0);
  EXPECT_EQ(shooter_output_fetcher_->voltage_right(), 0.0);
}

// Tests that the shooter spins up to speed and that it then spins down
// without applying any power.
TEST_F(ShooterTest, SpinUpAndDown) {
  SetEnabled(true);
  // Spin up.
  {
    auto builder = shooter_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_angular_velocity(450.0);
    EXPECT_TRUE(builder.Send(goal_builder.Finish()));
  }

  RunFor(chrono::seconds(1));
  VerifyNearGoal();
  shooter_status_fetcher_.Fetch();
  EXPECT_TRUE(shooter_status_fetcher_->ready());
  {
    auto builder = shooter_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_angular_velocity(0.0);
    EXPECT_TRUE(builder.Send(goal_builder.Finish()));
  }

  // Make sure we don't apply voltage on spin-down.
  RunFor(dt());
  EXPECT_TRUE(shooter_output_fetcher_.Fetch());
  EXPECT_EQ(0.0, shooter_output_fetcher_->voltage_left());
  EXPECT_EQ(0.0, shooter_output_fetcher_->voltage_right());

  // Continue to stop.
  RunFor(chrono::seconds(5));
  EXPECT_TRUE(shooter_output_fetcher_.Fetch());
  EXPECT_EQ(0.0, shooter_output_fetcher_->voltage_left());
  EXPECT_EQ(0.0, shooter_output_fetcher_->voltage_right());
}

// Tests that the shooter doesn't say it is ready if one side isn't up to speed.
// According to our tuning, we may overshoot the goal on one shooter and
// mistakenly say that we are ready. This test should look at both extremes.
TEST_F(ShooterTest, SideLagTest) {
  SetEnabled(true);
  // Spin up.
  {
    auto builder = shooter_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_angular_velocity(20.0);
    EXPECT_TRUE(builder.Send(goal_builder.Finish()));
  }
  // Cause problems by adding a voltage error on one side.
  shooter_plant_.set_right_voltage_offset(-4.0);
  RunFor(chrono::milliseconds(100));

  shooter_goal_fetcher_.Fetch();
  shooter_status_fetcher_.Fetch();

  EXPECT_TRUE(shooter_goal_fetcher_.get() != nullptr);
  EXPECT_TRUE(shooter_status_fetcher_.get() != nullptr);

  // Left should be up to speed, right shouldn't.
  EXPECT_TRUE(shooter_status_fetcher_->left()->ready());
  EXPECT_FALSE(shooter_status_fetcher_->right()->ready());
  EXPECT_FALSE(shooter_status_fetcher_->ready());

  RunFor(chrono::seconds(5));

  shooter_goal_fetcher_.Fetch();
  shooter_status_fetcher_.Fetch();

  EXPECT_TRUE(shooter_goal_fetcher_.get() != nullptr);
  EXPECT_TRUE(shooter_status_fetcher_.get() != nullptr);

  // Both should be up to speed.
  EXPECT_TRUE(shooter_status_fetcher_->left()->ready());
  EXPECT_TRUE(shooter_status_fetcher_->right()->ready());
  EXPECT_TRUE(shooter_status_fetcher_->ready());
}

// Tests that the shooter can spin up nicely after being disabled for a while.
TEST_F(ShooterTest, Disabled) {
  {
    auto builder = shooter_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_angular_velocity(200.0);
    EXPECT_TRUE(builder.Send(goal_builder.Finish()));
  }
  RunFor(chrono::seconds(5));
  EXPECT_TRUE(shooter_output_fetcher_.Fetch());
  EXPECT_EQ(shooter_output_fetcher_->voltage_left(), 0.0);
  EXPECT_EQ(shooter_output_fetcher_->voltage_right(), 0.0);

  SetEnabled(true);
  RunFor(chrono::seconds(5));

  VerifyNearGoal();
}

}  // namespace testing
}  // namespace shooter
}  // namespace control_loops
}  // namespace y2016
