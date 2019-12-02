#include <unistd.h>

#include <chrono>
#include <memory>

#include "aos/controls/control_loop_test.h"
#include "aos/network/team_number.h"
#include "frc971/control_loops/team_number_test_environment.h"
#include "gtest/gtest.h"
#include "y2014/constants.h"
#include "y2014/control_loops/shooter/shooter.h"
#include "y2014/control_loops/shooter/shooter_goal_generated.h"
#include "y2014/control_loops/shooter/shooter_output_generated.h"
#include "y2014/control_loops/shooter/shooter_position_generated.h"
#include "y2014/control_loops/shooter/shooter_status_generated.h"
#include "y2014/control_loops/shooter/unaugmented_shooter_motor_plant.h"

namespace chrono = ::std::chrono;
using ::aos::monotonic_clock;

namespace y2014 {
namespace control_loops {
namespace shooter {
namespace testing {

using ::y2014::control_loops::shooter::kMaxExtension;
using ::y2014::control_loops::shooter::MakeRawShooterPlant;
using ::frc971::control_loops::testing::kTeamNumber;

// Class which simulates the shooter and sends out queue messages containing the
// position.
class ShooterSimulation {
 public:
  // Constructs a motor simulation.
  ShooterSimulation(::aos::EventLoop *event_loop, chrono::nanoseconds dt,
                    double initial_position)
      : event_loop_(event_loop),
        shooter_position_sender_(event_loop_->MakeSender<Position>("/shooter")),
        shooter_output_fetcher_(event_loop_->MakeFetcher<Output>("/shooter")),
        shooter_plant_(new StateFeedbackPlant<2, 1, 1>(MakeRawShooterPlant())),
        latch_piston_state_(false),
        latch_delay_count_(0),
        plunger_latched_(false),
        brake_piston_state_(true),
        brake_delay_count_(0) {
    Reinitialize(initial_position);

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

  // The difference between the position with 0 at the back, and the position
  // with 0 measured where the spring has 0 force.
  constexpr static double kPositionOffset = kMaxExtension;

  void Reinitialize(double initial_position) {
    AOS_LOG(INFO, "Reinitializing to {pos: %f}\n", initial_position);
    StateFeedbackPlant<2, 1, 1> *plant = shooter_plant_.get();
    initial_position_ = initial_position;
    plant->mutable_X(0, 0) = initial_position_ - kPositionOffset;
    plant->mutable_X(1, 0) = 0.0;
    plant->mutable_Y() = plant->C() * plant->X();
    last_voltage_ = 0.0;
    last_plant_position_ = 0.0;
    last_position_ = 0.0;
    SetPhysicalSensors(&last_position_message_);
  }

  // Returns the absolute angle of the shooter.
  double GetAbsolutePosition() const {
    return shooter_plant_->Y(0, 0) + kPositionOffset;
  }

  // Returns the adjusted angle of the shooter.
  double GetPosition() const {
    return GetAbsolutePosition() - initial_position_;
  }

  // Makes sure pos is inside range (inclusive)
  bool CheckRange(double pos, struct constants::Values::AnglePair pair) {
    return (pos >= pair.lower_angle && pos <= pair.upper_angle);
  }

  // Sets the values of the physical sensors that can be directly observed
  // (encoder, hall effect).
  void SetPhysicalSensors(PositionT *position) {
    const constants::Values &values = constants::GetValues();

    position->position = GetPosition();

    AOS_LOG(DEBUG, "Physical shooter at {%f}\n", GetAbsolutePosition());

    // Signal that the hall effect sensor has been triggered if it is within
    // the correct range.
    if (plunger_latched_) {
      position->plunger = true;
      // Only disengage the spring if we are greater than 0, which is where the
      // latch will take the load off the pusher.
      if (GetAbsolutePosition() > 0.0) {
        shooter_plant_->set_index(1);
      } else {
        shooter_plant_->set_index(0);
      }
    } else {
      shooter_plant_->set_index(0);
      position->plunger =
          CheckRange(GetAbsolutePosition(), values.shooter.plunger_back);
    }

    if (!position->pusher_distal) {
      position->pusher_distal.reset(
          new frc971::PosedgeOnlyCountedHallEffectStructT);
    }
    if (!position->pusher_proximal) {
      position->pusher_proximal.reset(
          new frc971::PosedgeOnlyCountedHallEffectStructT);
    }

    position->pusher_distal->current =
        CheckRange(GetAbsolutePosition(), values.shooter.pusher_distal);
    position->pusher_proximal->current =
        CheckRange(GetAbsolutePosition(), values.shooter.pusher_proximal);
  }

  void UpdateEffectEdge(
      ::frc971::PosedgeOnlyCountedHallEffectStructT *sensor,
      const ::frc971::PosedgeOnlyCountedHallEffectStructT &last_sensor,
      const constants::Values::AnglePair &limits,
      float last_position) {
    sensor->posedge_count = last_sensor.posedge_count;
    sensor->negedge_count = last_sensor.negedge_count;

    sensor->posedge_value = last_sensor.posedge_value;

    if (sensor->current && !last_sensor.current) {
      ++sensor->posedge_count;
      if (last_position + initial_position_ < limits.lower_angle) {
        AOS_LOG(DEBUG,
                "Posedge value on lower edge of sensor, count is now %d\n",
                sensor->posedge_count);
        sensor->posedge_value = limits.lower_angle - initial_position_;
      } else {
        AOS_LOG(DEBUG,
                "Posedge value on upper edge of sensor, count is now %d\n",
                sensor->posedge_count);
        sensor->posedge_value = limits.upper_angle - initial_position_;
      }
    }
    if (!sensor->current && last_sensor.current) {
      ++sensor->negedge_count;
    }
  }

  void set_use_passed(bool use_passed) { use_passed_ = use_passed; }
  void set_plunger_in(bool plunger_in) { plunger_in_ = plunger_in; }
  void set_latch_in(bool latch_in) { latch_in_ = latch_in; }
  void set_brake_in(bool brake_in) { brake_in_ = brake_in; }

  // Sends out the position queue messages.
  // if used_passed_ is false then this is
  // just the default state, otherwise will force
  // it into a state using the plunger_in_, brake_in_, and latch_in_ values.
  void SendPositionMessage() {
    const constants::Values &values = constants::GetValues();
    ::aos::Sender<Position>::Builder builder =
        shooter_position_sender_.MakeBuilder();

    PositionT position;

    position.pusher_distal.reset(
        new frc971::PosedgeOnlyCountedHallEffectStructT);
    position.pusher_proximal.reset(
        new frc971::PosedgeOnlyCountedHallEffectStructT);

    if (use_passed_) {
      plunger_latched_ = latch_in_ && plunger_in_;
      latch_piston_state_ = plunger_latched_;
      brake_piston_state_ = brake_in_;
    }

    SetPhysicalSensors(&position);

    position.latch = latch_piston_state_;

    // Handle pusher distal hall effect
    UpdateEffectEdge(position.pusher_distal.get(),
                     *last_position_message_.pusher_distal.get(),
                     values.shooter.pusher_distal, last_position_);

    // Handle pusher proximal hall effect
    UpdateEffectEdge(position.pusher_proximal.get(),
                     *last_position_message_.pusher_proximal.get(),
                     values.shooter.pusher_proximal, last_position_);

    builder.Send(Position::Pack(*builder.fbb(), &position));

    last_position_ = position.position;
    last_position_message_ = std::move(position);
  }

  // Simulates the claw moving for one timestep.
  void Simulate() {
    last_plant_position_ = GetAbsolutePosition();
    EXPECT_TRUE(shooter_output_fetcher_.Fetch());
    if (shooter_output_fetcher_->latch_piston() && !latch_piston_state_ &&
        latch_delay_count_ <= 0) {
      ASSERT_EQ(0, latch_delay_count_) << "The test doesn't support that.";
      latch_delay_count_ = 6;
    } else if (!shooter_output_fetcher_->latch_piston() &&
               latch_piston_state_ && latch_delay_count_ >= 0) {
      ASSERT_EQ(0, latch_delay_count_) << "The test doesn't support that.";
      latch_delay_count_ = -6;
    }

    if (shooter_output_fetcher_->brake_piston() && !brake_piston_state_ &&
        brake_delay_count_ <= 0) {
      ASSERT_EQ(0, brake_delay_count_) << "The test doesn't support that.";
      brake_delay_count_ = 5;
    } else if (!shooter_output_fetcher_->brake_piston() &&
               brake_piston_state_ && brake_delay_count_ >= 0) {
      ASSERT_EQ(0, brake_delay_count_) << "The test doesn't support that.";
      brake_delay_count_ = -5;
    }

    // Handle brake internal state
    if (!brake_piston_state_ && brake_delay_count_ > 0) {
      if (brake_delay_count_ == 1) {
        brake_piston_state_ = true;
      }
      brake_delay_count_--;
    } else if (brake_piston_state_ && brake_delay_count_ < 0) {
      if (brake_delay_count_ == -1) {
        brake_piston_state_ = false;
      }
      brake_delay_count_++;
    }

    if (brake_piston_state_) {
      shooter_plant_->mutable_X(1, 0) = 0.0;
      shooter_plant_->mutable_Y() = shooter_plant_->C() * shooter_plant_->X();
    } else {
      Eigen::Matrix<double, 1, 1> U;
      U << last_voltage_;
      shooter_plant_->Update(U);
    }
    AOS_LOG(DEBUG, "Plant index is %d\n", shooter_plant_->index());

    // Handle latch hall effect
    if (!latch_piston_state_ && latch_delay_count_ > 0) {
      AOS_LOG(DEBUG, "latching simulation: %dp\n", latch_delay_count_);
      if (latch_delay_count_ == 1) {
        latch_piston_state_ = true;
        EXPECT_GE(constants::GetValues().shooter.latch_max_safe_position,
                  GetAbsolutePosition());
        plunger_latched_ = true;
      }
      latch_delay_count_--;
    } else if (latch_piston_state_ && latch_delay_count_ < 0) {
      AOS_LOG(DEBUG, "latching simulation: %dn\n", latch_delay_count_);
      if (latch_delay_count_ == -1) {
        latch_piston_state_ = false;
        if (GetAbsolutePosition() > 0.002) {
          EXPECT_TRUE(brake_piston_state_) << "Must have the brake set when "
                                              "releasing the latch for "
                                              "powerful shots.";
        }
        plunger_latched_ = false;
        // TODO(austin): The brake should be set for a number of cycles after
        // this as well.
        shooter_plant_->mutable_X(0, 0) += 0.005;
      }
      latch_delay_count_++;
    }

    EXPECT_GE(constants::GetValues().shooter.upper_hard_limit,
              GetAbsolutePosition());
    EXPECT_LE(constants::GetValues().shooter.lower_hard_limit,
              GetAbsolutePosition());

    last_voltage_ = shooter_output_fetcher_->voltage();
  }

  private:
  ::aos::EventLoop *event_loop_;
  ::aos::Sender<Position> shooter_position_sender_;
  ::aos::Fetcher<Output> shooter_output_fetcher_;

  bool first_ = true;

  // pointer to plant
  const ::std::unique_ptr<StateFeedbackPlant<2, 1, 1>> shooter_plant_;

  // true latch closed
  bool latch_piston_state_;
  // greater than zero, delaying close. less than zero delaying open
  int latch_delay_count_;

  // Goes to true after latch_delay_count_ hits 0 while the plunger is back.
  bool plunger_latched_;

  // true brake locked
  bool brake_piston_state_;
  // greater than zero, delaying close. less than zero delaying open
  int brake_delay_count_;

  // Overrides for testing.
  bool use_passed_ = false;
  bool plunger_in_ = false;
  bool latch_in_ = false;
  bool brake_in_ = false;

  double initial_position_;
  double last_voltage_;

  double last_position_ = 0.0;
  PositionT last_position_message_;
  double last_plant_position_;
};

template <typename TestType>
class ShooterTestTemplated
    : public ::aos::testing::ControlLoopTestTemplated<TestType> {
 protected:
  ShooterTestTemplated()
      : ::aos::testing::ControlLoopTestTemplated<TestType>(
            aos::configuration::ReadConfig("y2014/config.json"),
            // TODO(austin): I think this runs at 5 ms in real life.
            chrono::microseconds(5000)),
        test_event_loop_(this->MakeEventLoop("test")),
        shooter_goal_fetcher_(test_event_loop_->MakeFetcher<Goal>("/shooter")),
        shooter_goal_sender_(test_event_loop_->MakeSender<Goal>("/shooter")),
        shooter_event_loop_(this->MakeEventLoop("shooter")),
        shooter_motor_(shooter_event_loop_.get()),
        shooter_plant_event_loop_(this->MakeEventLoop("plant")),
        shooter_motor_plant_(shooter_plant_event_loop_.get(), this->dt(), 0.2) {
  }

  void Reinitialize(double position) {
    shooter_motor_plant_.Reinitialize(position);
  }

  void VerifyNearGoal() {
    shooter_goal_fetcher_.Fetch();
    const double pos = shooter_motor_plant_.GetAbsolutePosition();
    EXPECT_NEAR(shooter_goal_fetcher_->shot_power(), pos, 1e-4);
  }

  ::std::unique_ptr<::aos::EventLoop> test_event_loop_;
  ::aos::Fetcher<Goal> shooter_goal_fetcher_;
  ::aos::Sender<Goal> shooter_goal_sender_;

  // Create a loop and simulation plant.
  ::std::unique_ptr<::aos::EventLoop> shooter_event_loop_;
  ShooterMotor shooter_motor_;
  ::std::unique_ptr<::aos::EventLoop> shooter_plant_event_loop_;
  ShooterSimulation shooter_motor_plant_;
};

typedef ShooterTestTemplated<::testing::Test> ShooterTest;

TEST_F(ShooterTest, PowerConversion) {
  const constants::Values &values = constants::GetValues();
  // test a couple of values return the right thing
  EXPECT_NEAR(0.254001, shooter_motor_.PowerToPosition(140.0), 0.00001);
  EXPECT_NEAR(0.00058, shooter_motor_.PowerToPosition(0.53), 0.00001);
  EXPECT_NEAR(0.095251238129837101, shooter_motor_.PowerToPosition(73.67),
              0.00001);

  // value too large should get max
  EXPECT_NEAR(values.shooter.upper_limit,
              shooter_motor_.PowerToPosition(505050.99), 0.00001);
  // negative values should zero
  EXPECT_NEAR(0, shooter_motor_.PowerToPosition(-123.4), 0.00001);
}

// Test that PowerToPosition and PositionToPower are inverses of each other.
// Note that PowerToPosition will cap position whereas PositionToPower will not
// cap power.
TEST_F(ShooterTest, InversePowerConversion) {
  // Test a few values.
  double power = 140.0;
  double position = shooter_motor_.PowerToPosition(power);
  EXPECT_NEAR(power, shooter_motor_.PositionToPower(position), 1e-5);
  power = .53;
  position = shooter_motor_.PowerToPosition(power);
  EXPECT_NEAR(power, shooter_motor_.PositionToPower(position), 1e-5);
  power = 71.971;
  position = shooter_motor_.PowerToPosition(power);
  EXPECT_NEAR(power, shooter_motor_.PositionToPower(position), 1e-5);
}

// Tests that the shooter zeros correctly and goes to a position.
TEST_F(ShooterTest, GoesToValue) {
  SetEnabled(true);
  {
    ::aos::Sender<Goal>::Builder builder = shooter_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_shot_power(70.0);
    EXPECT_TRUE(builder.Send(goal_builder.Finish()));
  }
  RunFor(chrono::seconds(2));

  double pos = shooter_motor_plant_.GetAbsolutePosition();
  EXPECT_TRUE(shooter_goal_fetcher_.Fetch());
  EXPECT_NEAR(
      shooter_motor_.PowerToPosition(shooter_goal_fetcher_->shot_power()), pos,
      0.05);
  EXPECT_EQ(ShooterMotor::STATE_READY, shooter_motor_.state());
}

// Tests that the shooter zeros correctly and goes to a position.
TEST_F(ShooterTest, Fire) {
  SetEnabled(true);
  {
    ::aos::Sender<Goal>::Builder builder = shooter_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_shot_power(70.0);
    EXPECT_TRUE(builder.Send(goal_builder.Finish()));
  }
  RunFor(chrono::milliseconds(1200));
  EXPECT_EQ(ShooterMotor::STATE_READY, shooter_motor_.state());
  {
    ::aos::Sender<Goal>::Builder builder = shooter_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_shot_power(35.0);
    goal_builder.add_shot_requested(true);
    EXPECT_TRUE(builder.Send(goal_builder.Finish()));
  }

  bool hit_fire = false;
  while (test_event_loop_->monotonic_now() <
         monotonic_clock::time_point(chrono::milliseconds(5200))) {
    RunFor(dt());
    if (shooter_motor_.state() == ShooterMotor::STATE_FIRE) {
      if (!hit_fire) {
        ::aos::Sender<Goal>::Builder builder =
            shooter_goal_sender_.MakeBuilder();
        Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
        goal_builder.add_shot_power(17.0);
        goal_builder.add_shot_requested(false);
        EXPECT_TRUE(builder.Send(goal_builder.Finish()));
      }
      hit_fire = true;
    }
  }

  double pos = shooter_motor_plant_.GetAbsolutePosition();
  EXPECT_TRUE(shooter_goal_fetcher_.Fetch());
  EXPECT_NEAR(
      shooter_motor_.PowerToPosition(shooter_goal_fetcher_->shot_power()), pos,
      0.05);
  EXPECT_EQ(ShooterMotor::STATE_READY, shooter_motor_.state());
  EXPECT_TRUE(hit_fire);
}

// Tests that the shooter zeros correctly and goes to a position.
TEST_F(ShooterTest, FireLong) {
  SetEnabled(true);
  {
    ::aos::Sender<Goal>::Builder builder = shooter_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_shot_power(70.0);
    EXPECT_TRUE(builder.Send(goal_builder.Finish()));
  }
  RunFor(chrono::milliseconds(1500));

  EXPECT_EQ(ShooterMotor::STATE_READY, shooter_motor_.state());
  {
    ::aos::Sender<Goal>::Builder builder = shooter_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_shot_power(0.0);
    goal_builder.add_shot_requested(true);
    EXPECT_TRUE(builder.Send(goal_builder.Finish()));
  }

  bool hit_fire = false;
  while (test_event_loop_->monotonic_now() <
         monotonic_clock::time_point(chrono::milliseconds(5500))) {
    RunFor(dt());
    if (shooter_motor_.state() == ShooterMotor::STATE_FIRE) {
      if (!hit_fire) {
        ::aos::Sender<Goal>::Builder builder =
            shooter_goal_sender_.MakeBuilder();
        Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
        goal_builder.add_shot_requested(false);
        EXPECT_TRUE(builder.Send(goal_builder.Finish()));
      }
      hit_fire = true;
    }
  }

  double pos = shooter_motor_plant_.GetAbsolutePosition();
  EXPECT_TRUE(shooter_goal_fetcher_.Fetch());
  EXPECT_NEAR(
      shooter_motor_.PowerToPosition(shooter_goal_fetcher_->shot_power()), pos,
      0.05);
  EXPECT_EQ(ShooterMotor::STATE_READY, shooter_motor_.state());
  EXPECT_TRUE(hit_fire);
}

// Verifies that it doesn't try to go out too far if you give it a ridicilous
// power.
TEST_F(ShooterTest, LoadTooFar) {
  SetEnabled(true);
  {
    ::aos::Sender<Goal>::Builder builder = shooter_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_shot_power(500.0);
    EXPECT_TRUE(builder.Send(goal_builder.Finish()));
  }
  while (test_event_loop_->monotonic_now() <
         monotonic_clock::time_point(chrono::milliseconds(1600))) {
    RunFor(dt());
    EXPECT_LT(shooter_motor_plant_.GetAbsolutePosition(),
              constants::GetValuesForTeam(kTeamNumber).shooter.upper_limit);
  }
  EXPECT_EQ(ShooterMotor::STATE_READY, shooter_motor_.state());
}

// Tests that the shooter zeros correctly and goes to a position.
TEST_F(ShooterTest, MoveGoal) {
  SetEnabled(true);
  {
    ::aos::Sender<Goal>::Builder builder = shooter_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_shot_power(70.0);
    EXPECT_TRUE(builder.Send(goal_builder.Finish()));
  }
  RunFor(chrono::milliseconds(1500));

  EXPECT_EQ(ShooterMotor::STATE_READY, shooter_motor_.state());
  {
    ::aos::Sender<Goal>::Builder builder = shooter_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_shot_power(14.0);
    EXPECT_TRUE(builder.Send(goal_builder.Finish()));
  }

  RunFor(chrono::milliseconds(500));

  double pos = shooter_motor_plant_.GetAbsolutePosition();
  EXPECT_TRUE(shooter_goal_fetcher_.Fetch());
  EXPECT_NEAR(
      shooter_motor_.PowerToPosition(shooter_goal_fetcher_->shot_power()), pos,
      0.05);
  EXPECT_EQ(ShooterMotor::STATE_READY, shooter_motor_.state());
}


TEST_F(ShooterTest, Unload) {
  SetEnabled(true);
  {
    ::aos::Sender<Goal>::Builder builder = shooter_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_shot_power(70.0);
    EXPECT_TRUE(builder.Send(goal_builder.Finish()));
  }
  RunFor(chrono::milliseconds(1500));
  EXPECT_EQ(ShooterMotor::STATE_READY, shooter_motor_.state());
  {
    ::aos::Sender<Goal>::Builder builder = shooter_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_unload_requested(true);
    EXPECT_TRUE(builder.Send(goal_builder.Finish()));
  }

  while (test_event_loop_->monotonic_now() <
             monotonic_clock::time_point(chrono::seconds(8)) &&
         shooter_motor_.state() != ShooterMotor::STATE_READY_UNLOAD) {
    RunFor(dt());
  }

  EXPECT_NEAR(constants::GetValues().shooter.upper_limit,
              shooter_motor_plant_.GetAbsolutePosition(), 0.015);
  EXPECT_EQ(ShooterMotor::STATE_READY_UNLOAD, shooter_motor_.state());
}

// Tests that it rezeros while unloading.
TEST_F(ShooterTest, RezeroWhileUnloading) {
  SetEnabled(true);
  {
    ::aos::Sender<Goal>::Builder builder = shooter_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_shot_power(70);
    EXPECT_TRUE(builder.Send(goal_builder.Finish()));
  }
  RunFor(chrono::milliseconds(1500));

  EXPECT_EQ(ShooterMotor::STATE_READY, shooter_motor_.state());

  shooter_motor_.shooter_.offset_ += 0.01;
  RunFor(chrono::milliseconds(500));

  {
    ::aos::Sender<Goal>::Builder builder = shooter_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_unload_requested(true);
    EXPECT_TRUE(builder.Send(goal_builder.Finish()));
  }

  while (test_event_loop_->monotonic_now() <
             ::aos::monotonic_clock::time_point(chrono::seconds(10)) &&
         shooter_motor_.state() != ShooterMotor::STATE_READY_UNLOAD) {
    RunFor(dt());
  }

  EXPECT_NEAR(constants::GetValues().shooter.upper_limit,
              shooter_motor_plant_.GetAbsolutePosition(), 0.015);
  EXPECT_EQ(ShooterMotor::STATE_READY_UNLOAD, shooter_motor_.state());
}

// Tests that the shooter zeros correctly and goes to a position.
TEST_F(ShooterTest, UnloadWindupNegative) {
  SetEnabled(true);
  {
    ::aos::Sender<Goal>::Builder builder = shooter_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_shot_power(70);
    EXPECT_TRUE(builder.Send(goal_builder.Finish()));
  }
  RunFor(chrono::milliseconds(1500));
  EXPECT_EQ(ShooterMotor::STATE_READY, shooter_motor_.state());
  {
    ::aos::Sender<Goal>::Builder builder = shooter_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_unload_requested(true);
    EXPECT_TRUE(builder.Send(goal_builder.Finish()));
  }

  int kicked_delay = 20;
  int capped_goal_count = 0;
  while (test_event_loop_->monotonic_now() <
             monotonic_clock::time_point(chrono::milliseconds(9500)) &&
         shooter_motor_.state() != ShooterMotor::STATE_READY_UNLOAD) {
    RunFor(dt());
    if (shooter_motor_.state() == ShooterMotor::STATE_UNLOAD_MOVE) {
      AOS_LOG(DEBUG, "State is UnloadMove\n");
      --kicked_delay;
      if (kicked_delay == 0) {
        shooter_motor_.shooter_.mutable_R(0, 0) -= 100;
      }
    }
    if (shooter_motor_.capped_goal() && kicked_delay < 0) {
      ++capped_goal_count;
    }
  }

  EXPECT_NEAR(constants::GetValues().shooter.upper_limit,
              shooter_motor_plant_.GetAbsolutePosition(), 0.05);
  EXPECT_EQ(ShooterMotor::STATE_READY_UNLOAD, shooter_motor_.state());
  EXPECT_LE(1, capped_goal_count);
  EXPECT_GE(3, capped_goal_count);
}

// Tests that the shooter zeros correctly and goes to a position.
TEST_F(ShooterTest, UnloadWindupPositive) {
  SetEnabled(true);
  {
    ::aos::Sender<Goal>::Builder builder = shooter_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_shot_power(70);
    EXPECT_TRUE(builder.Send(goal_builder.Finish()));
  }
  RunFor(chrono::milliseconds(1500));
  EXPECT_EQ(ShooterMotor::STATE_READY, shooter_motor_.state());
  {
    ::aos::Sender<Goal>::Builder builder = shooter_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_unload_requested(true);
    EXPECT_TRUE(builder.Send(goal_builder.Finish()));
  }

  int kicked_delay = 20;
  int capped_goal_count = 0;
  while (test_event_loop_->monotonic_now() <
             monotonic_clock::time_point(chrono::milliseconds(9500)) &&
         shooter_motor_.state() != ShooterMotor::STATE_READY_UNLOAD) {
    RunFor(dt());
    if (shooter_motor_.state() == ShooterMotor::STATE_UNLOAD_MOVE) {
      AOS_LOG(DEBUG, "State is UnloadMove\n");
      --kicked_delay;
      if (kicked_delay == 0) {
        shooter_motor_.shooter_.mutable_R(0, 0) += 0.1;
      }
    }
    if (shooter_motor_.capped_goal() && kicked_delay < 0) {
      ++capped_goal_count;
    }
  }

  EXPECT_NEAR(constants::GetValues().shooter.upper_limit,
              shooter_motor_plant_.GetAbsolutePosition(), 0.05);
  EXPECT_EQ(ShooterMotor::STATE_READY_UNLOAD, shooter_motor_.state());
  EXPECT_LE(1, capped_goal_count);
  EXPECT_GE(3, capped_goal_count);
}

double HallEffectMiddle(constants::Values::AnglePair pair) {
  return (pair.lower_angle + pair.upper_angle) / 2.0;
}

// Tests that the shooter zeros correctly and goes to a position.
TEST_F(ShooterTest, StartsOnDistal) {
  SetEnabled(true);
  Reinitialize(HallEffectMiddle(constants::GetValues().shooter.pusher_distal));
  {
    ::aos::Sender<Goal>::Builder builder = shooter_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_shot_power(70.0);
    EXPECT_TRUE(builder.Send(goal_builder.Finish()));
  }
  RunFor(chrono::seconds(2));
  // EXPECT_NEAR(0.0, shooter_motor_.GetPosition(), 0.01);
  double pos = shooter_motor_plant_.GetAbsolutePosition();
  EXPECT_TRUE(shooter_goal_fetcher_.Fetch());
  EXPECT_NEAR(
      shooter_motor_.PowerToPosition(shooter_goal_fetcher_->shot_power()), pos,
      0.05);
  EXPECT_EQ(ShooterMotor::STATE_READY, shooter_motor_.state());
}


// Tests that the shooter zeros correctly and goes to a position.
TEST_F(ShooterTest, StartsOnProximal) {
  SetEnabled(true);
  Reinitialize(
      HallEffectMiddle(constants::GetValues().shooter.pusher_proximal));
  {
    ::aos::Sender<Goal>::Builder builder = shooter_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_shot_power(70.0);
    EXPECT_TRUE(builder.Send(goal_builder.Finish()));
  }
  RunFor(chrono::seconds(3));
  // EXPECT_NEAR(0.0, shooter_motor_.GetPosition(), 0.01);
  double pos = shooter_motor_plant_.GetAbsolutePosition();
  EXPECT_TRUE(shooter_goal_fetcher_.Fetch());
  EXPECT_NEAR(
      shooter_motor_.PowerToPosition(shooter_goal_fetcher_->shot_power()), pos,
      0.05);
  EXPECT_EQ(ShooterMotor::STATE_READY, shooter_motor_.state());
}

class ShooterZeroingTest
    : public ShooterTestTemplated<
          ::testing::TestWithParam<::std::tuple<bool, bool, bool, double>>> {};

TEST_P(ShooterZeroingTest, AllDisparateStartingZero) {
  SetEnabled(true);
  bool latch = ::std::get<0>(GetParam());
  bool brake = ::std::get<1>(GetParam());
  bool plunger_back = ::std::get<2>(GetParam());
  double start_pos = ::std::get<3>(GetParam());
  // flag to initialize test
	//printf("@@@@ l= %d b= %d p= %d s= %.3f\n",
	//		latch, brake, plunger_back, start_pos);
  bool initialized = false;
  Reinitialize(start_pos);
  {
    ::aos::Sender<Goal>::Builder builder = shooter_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_shot_power(120.0);
    EXPECT_TRUE(builder.Send(goal_builder.Finish()));
  }
  while (test_event_loop_->monotonic_now() <
         monotonic_clock::time_point(chrono::seconds(2))) {
    shooter_motor_plant_.set_use_passed(!initialized);
    shooter_motor_plant_.set_plunger_in(plunger_back);
    shooter_motor_plant_.set_latch_in(latch);
    shooter_motor_plant_.set_brake_in(brake);
    initialized = true;
    RunFor(dt());
  }
  // EXPECT_NEAR(0.0, shooter_motor_.GetPosition(), 0.01);
  double pos = shooter_motor_plant_.GetAbsolutePosition();
  EXPECT_TRUE(shooter_goal_fetcher_.Fetch());
  EXPECT_NEAR(
      shooter_motor_.PowerToPosition(shooter_goal_fetcher_->shot_power()), pos,
      0.05);
  ASSERT_EQ(ShooterMotor::STATE_READY, shooter_motor_.state());
}

INSTANTIATE_TEST_CASE_P(
    ShooterZeroingTestParameters, ShooterZeroingTest,
    ::testing::Combine(
        ::testing::Bool(), ::testing::Bool(), ::testing::Bool(),
        ::testing::Values(
            0.05,
            constants::GetValuesForTeam(kTeamNumber).shooter.upper_limit - 0.05,
            HallEffectMiddle(constants::GetValuesForTeam(kTeamNumber)
                                 .shooter.pusher_proximal),
            HallEffectMiddle(
                constants::GetValuesForTeam(kTeamNumber).shooter.pusher_distal),
            constants::GetValuesForTeam(kTeamNumber)
                    .shooter.latch_max_safe_position -
                0.001)));

// TODO(austin): Slip the encoder somewhere.

// TODO(austin): Test all the timeouts...

}  // namespace testing
}  // namespace shooter
}  // namespace control_loops
}  // namespace y2014
