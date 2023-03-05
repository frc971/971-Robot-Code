#include <chrono>
#include <memory>

#include "aos/events/logging/log_writer.h"
#include "frc971/control_loops/capped_test_plant.h"
#include "frc971/control_loops/control_loop_test.h"
#include "frc971/control_loops/position_sensor_sim.h"
#include "frc971/control_loops/subsystem_simulator.h"
#include "frc971/control_loops/team_number_test_environment.h"
#include "gtest/gtest.h"
#include "y2023/control_loops/drivetrain/drivetrain_dog_motor_plant.h"
#include "y2023/control_loops/superstructure/roll/integral_roll_plant.h"
#include "y2023/control_loops/superstructure/superstructure.h"

DEFINE_string(output_folder, "",
              "If set, logs all channels to the provided logfile.");

namespace y2023 {
namespace control_loops {
namespace superstructure {
namespace testing {
namespace {
constexpr double kNoiseScalar = 0.01;
}  // namespace
namespace chrono = std::chrono;

using ::aos::monotonic_clock;
using ::frc971::CreateProfileParameters;
using ::frc971::control_loops::CappedTestPlant;
using ::frc971::control_loops::
    CreateStaticZeroingSingleDOFProfiledSubsystemGoal;
using ::frc971::control_loops::PositionSensorSimulator;
using ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoal;
typedef Superstructure::PotAndAbsoluteEncoderSubsystem
    PotAndAbsoluteEncoderSubsystem;
typedef Superstructure::RelativeEncoderSubsystem RelativeEncoderSubsystem;
using DrivetrainStatus = ::frc971::control_loops::drivetrain::Status;
using PotAndAbsoluteEncoderSimulator =
    frc971::control_loops::SubsystemSimulator<
        frc971::control_loops::PotAndAbsoluteEncoderProfiledJointStatus,
        PotAndAbsoluteEncoderSubsystem::State,
        constants::Values::PotAndAbsEncoderConstants>;
using RelativeEncoderSimulator = frc971::control_loops::SubsystemSimulator<
    frc971::control_loops::RelativeEncoderProfiledJointStatus,
    RelativeEncoderSubsystem::State, constants::Values::PotConstants>;
using AbsoluteEncoderSubsystem =
    ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystem<
        ::frc971::zeroing::AbsoluteEncoderZeroingEstimator,
        ::frc971::control_loops::AbsoluteEncoderProfiledJointStatus>;

using AbsoluteEncoderSimulator = ::frc971::control_loops::SubsystemSimulator<
    ::frc971::control_loops::AbsoluteEncoderProfiledJointStatus,
    Superstructure::AbsoluteEncoderSubsystem::State,
    constants::Values::AbsEncoderConstants>;

class ArmSimulation {
 public:
  explicit ArmSimulation(
      const ::frc971::constants::PotAndAbsoluteEncoderZeroingConstants
          &proximal_zeroing_constants,
      const ::frc971::constants::PotAndAbsoluteEncoderZeroingConstants
          &distal_zeroing_constants,
      const ::frc971::constants::PotAndAbsoluteEncoderZeroingConstants
          &roll_joint_zeroing_constants,
      std::chrono::nanoseconds dt)
      : proximal_zeroing_constants_(proximal_zeroing_constants),
        proximal_pot_encoder_(M_PI * 2.0 *
                              constants::Values::kProximalEncoderRatio()),
        distal_zeroing_constants_(distal_zeroing_constants),
        distal_pot_encoder_(M_PI * 2.0 *
                            constants::Values::kDistalEncoderRatio()),
        roll_joint_zeroing_constants_(roll_joint_zeroing_constants),
        roll_joint_pot_encoder_(M_PI * 2.0 *
                                constants::Values::kDistalEncoderRatio()),
        roll_joint_loop_(roll::MakeIntegralRollLoop()),
        dynamics_(arm::kArmConstants),
        dt_(dt) {
    X_.setZero();
    roll_joint_loop_.Reset();
  }

  void InitializePosition(::Eigen::Matrix<double, 3, 1> position) {
    X_ << position(0), 0.0, position(1), 0.0;

    proximal_pot_encoder_.Initialize(
        X_(0), kNoiseScalar, 0.0,
        proximal_zeroing_constants_.measured_absolute_position);
    distal_pot_encoder_.Initialize(
        X_(2), kNoiseScalar, 0.0,
        distal_zeroing_constants_.measured_absolute_position);

    Eigen::Matrix<double, 3, 1> X_roll_joint;
    X_roll_joint << position(2), 0.0, 0.0;
    roll_joint_loop_.mutable_X_hat() = X_roll_joint;
    roll_joint_pot_encoder_.Initialize(
        X_roll_joint(0), kNoiseScalar, 0.0,
        roll_joint_zeroing_constants_.measured_absolute_position);
  }

  flatbuffers::Offset<ArmPosition> GetSensorValues(
      flatbuffers::FlatBufferBuilder *fbb) {
    frc971::PotAndAbsolutePosition::Builder proximal_builder(*fbb);
    flatbuffers::Offset<frc971::PotAndAbsolutePosition> proximal_offset =
        proximal_pot_encoder_.GetSensorValues(&proximal_builder);

    frc971::PotAndAbsolutePosition::Builder distal_builder(*fbb);
    flatbuffers::Offset<frc971::PotAndAbsolutePosition> distal_offset =
        distal_pot_encoder_.GetSensorValues(&distal_builder);

    frc971::PotAndAbsolutePosition::Builder roll_joint_builder(*fbb);
    flatbuffers::Offset<frc971::PotAndAbsolutePosition> roll_joint_offset =
        roll_joint_pot_encoder_.GetSensorValues(&roll_joint_builder);

    ArmPosition::Builder arm_position_builder(*fbb);
    arm_position_builder.add_proximal(proximal_offset);
    arm_position_builder.add_distal(distal_offset);
    arm_position_builder.add_roll_joint(roll_joint_offset);

    return arm_position_builder.Finish();
  }

  double proximal_position() const { return X_(0, 0); }
  double proximal_velocity() const { return X_(1, 0); }
  double distal_position() const { return X_(2, 0); }
  double distal_velocity() const { return X_(3, 0); }
  double roll_joint_position() const { return roll_joint_loop_.X_hat(0, 0); }
  double roll_joint_velocity() const { return roll_joint_loop_.X_hat(1, 0); }

  void Simulate(::Eigen::Matrix<double, 3, 1> U) {
    constexpr double voltage_check = constants::Values::kArmOperatingVoltage();

    AOS_CHECK_LE(::std::abs(U(0)), voltage_check);
    AOS_CHECK_LE(::std::abs(U(1)), voltage_check);
    AOS_CHECK_LE(::std::abs(U(2)), voltage_check);

    X_ = dynamics_.UnboundedDiscreteDynamics(
        X_, U.head<2>(),
        std::chrono::duration_cast<std::chrono::duration<double>>(dt_).count());
    roll_joint_loop_.UpdateObserver(U.tail<1>(), dt_);

    // TODO(austin): Estop on grose out of bounds.
    proximal_pot_encoder_.MoveTo(X_(0));
    distal_pot_encoder_.MoveTo(X_(2));
    roll_joint_pot_encoder_.MoveTo(roll_joint_loop_.X_hat(0));
  }

 private:
  ::Eigen::Matrix<double, 4, 1> X_;

  const ::frc971::constants::PotAndAbsoluteEncoderZeroingConstants
      proximal_zeroing_constants_;
  PositionSensorSimulator proximal_pot_encoder_;

  const ::frc971::constants::PotAndAbsoluteEncoderZeroingConstants
      distal_zeroing_constants_;
  PositionSensorSimulator distal_pot_encoder_;

  const ::frc971::constants::PotAndAbsoluteEncoderZeroingConstants
      roll_joint_zeroing_constants_;
  PositionSensorSimulator roll_joint_pot_encoder_;
  StateFeedbackLoop<3, 1, 1, double, StateFeedbackPlant<3, 1, 1>,
                    StateFeedbackObserver<3, 1, 1>>
      roll_joint_loop_;

  ::frc971::control_loops::arm::Dynamics dynamics_;

  std::chrono::nanoseconds dt_;
};
// Class which simulates the superstructure and sends out queue messages with
// the position.
class SuperstructureSimulation {
 public:
  SuperstructureSimulation(::aos::EventLoop *event_loop,
                           std::shared_ptr<const constants::Values> values,
                           chrono::nanoseconds dt)
      : event_loop_(event_loop),
        dt_(dt),
        arm_(values->arm_proximal.zeroing, values->arm_distal.zeroing,
             values->roll_joint.zeroing, dt_),
        wrist_(new CappedTestPlant(wrist::MakeWristPlant()),
               PositionSensorSimulator(
                   values->wrist.subsystem_params.zeroing_constants
                       .one_revolution_distance),
               values->wrist, constants::Values::kCompWristRange(),
               values->wrist.subsystem_params.zeroing_constants
                   .measured_absolute_position,
               dt_),
        superstructure_position_sender_(
            event_loop_->MakeSender<Position>("/superstructure")),
        superstructure_status_fetcher_(
            event_loop_->MakeFetcher<Status>("/superstructure")),
        superstructure_output_fetcher_(
            event_loop_->MakeFetcher<Output>("/superstructure")) {
    InitializeArmPosition(arm::NeutralPoint());
    phased_loop_handle_ = event_loop_->AddPhasedLoop(
        [this](int) {
          // Skip this the first time.
          if (!first_) {
            EXPECT_TRUE(superstructure_output_fetcher_.Fetch());
            EXPECT_TRUE(superstructure_status_fetcher_.Fetch());

            arm_.Simulate(
                (::Eigen::Matrix<double, 3, 1>()
                     << superstructure_output_fetcher_->proximal_voltage(),
                 superstructure_output_fetcher_->distal_voltage(),
                 superstructure_output_fetcher_->roll_joint_voltage())
                    .finished());

            wrist_.Simulate(superstructure_output_fetcher_->wrist_voltage(),
                            superstructure_status_fetcher_->wrist());
          }
          first_ = false;
          SendPositionMessage();
        },
        dt);
  }

  void InitializeArmPosition(::Eigen::Matrix<double, 3, 1> position) {
    arm_.InitializePosition(position);
  }

  ArmSimulation *arm() { return &arm_; }
  AbsoluteEncoderSimulator *wrist() { return &wrist_; }

  // Sends a queue message with the position of the superstructure.
  void SendPositionMessage() {
    ::aos::Sender<Position>::Builder builder =
        superstructure_position_sender_.MakeBuilder();

    flatbuffers::Offset<ArmPosition> arm_offset =
        arm_.GetSensorValues(builder.fbb());

    frc971::AbsolutePosition::Builder wrist_builder =
        builder.MakeBuilder<frc971::AbsolutePosition>();
    flatbuffers::Offset<frc971::AbsolutePosition> wrist_offset =
        wrist_.encoder()->GetSensorValues(&wrist_builder);

    Position::Builder position_builder = builder.MakeBuilder<Position>();
    position_builder.add_arm(arm_offset);
    position_builder.add_wrist(wrist_offset);
    position_builder.add_end_effector_cube_beam_break(
        end_effector_cube_beam_break_);
    // TODO(milind): put into our state
    position_builder.add_cone_position(0.95);
    CHECK_EQ(builder.Send(position_builder.Finish()),
             aos::RawSender::Error::kOk);
  }

  void set_end_effector_cube_beam_break(bool triggered) {
    end_effector_cube_beam_break_ = triggered;
  }

 private:
  ::aos::EventLoop *event_loop_;
  const chrono::nanoseconds dt_;
  ::aos::PhasedLoopHandler *phased_loop_handle_ = nullptr;

  ArmSimulation arm_;
  AbsoluteEncoderSimulator wrist_;

  bool end_effector_cube_beam_break_;

  ::aos::Sender<Position> superstructure_position_sender_;
  ::aos::Fetcher<Status> superstructure_status_fetcher_;
  ::aos::Fetcher<Output> superstructure_output_fetcher_;

  bool first_ = true;
};

class SuperstructureTest : public ::frc971::testing::ControlLoopTest {
 public:
  SuperstructureTest()
      : ::frc971::testing::ControlLoopTest(
            aos::configuration::ReadConfig("y2023/aos_config.json"),
            std::chrono::microseconds(5050)),
        values_(std::make_shared<constants::Values>(constants::MakeValues())),
        roborio_(aos::configuration::GetNode(configuration(), "roborio")),
        logger_pi_(aos::configuration::GetNode(configuration(), "logger")),
        arm_trajectories_(superstructure::Superstructure::GetArmTrajectories(
            "y2023/control_loops/superstructure/arm/"
            "arm_trajectories_generated.bfbs")),
        superstructure_event_loop(MakeEventLoop("Superstructure", roborio_)),
        superstructure_(superstructure_event_loop.get(), values_,
                        arm_trajectories_),
        test_event_loop_(MakeEventLoop("test", roborio_)),
        superstructure_goal_fetcher_(
            test_event_loop_->MakeFetcher<Goal>("/superstructure")),
        superstructure_goal_sender_(
            test_event_loop_->MakeSender<Goal>("/superstructure")),
        superstructure_status_fetcher_(
            test_event_loop_->MakeFetcher<Status>("/superstructure")),
        superstructure_output_fetcher_(
            test_event_loop_->MakeFetcher<Output>("/superstructure")),
        superstructure_position_fetcher_(
            test_event_loop_->MakeFetcher<Position>("/superstructure")),
        superstructure_position_sender_(
            test_event_loop_->MakeSender<Position>("/superstructure")),
        drivetrain_status_sender_(
            test_event_loop_->MakeSender<DrivetrainStatus>("/drivetrain")),
        superstructure_plant_event_loop_(MakeEventLoop("plant", roborio_)),
        superstructure_plant_(superstructure_plant_event_loop_.get(), values_,
                              dt()),
        points_(arm::PointList()) {
    set_team_id(frc971::control_loops::testing::kTeamNumber);

    SetEnabled(true);

    if (!FLAGS_output_folder.empty()) {
      unlink(FLAGS_output_folder.c_str());
      logger_event_loop_ = MakeEventLoop("logger", roborio_);
      logger_ = std::make_unique<aos::logger::Logger>(logger_event_loop_.get());
      logger_->StartLoggingOnRun(FLAGS_output_folder);
    }
  }

  void VerifyNearGoal() {
    superstructure_goal_fetcher_.Fetch();
    superstructure_status_fetcher_.Fetch();

    ASSERT_TRUE(superstructure_goal_fetcher_.get() != nullptr) << ": No goal";
    ASSERT_TRUE(superstructure_status_fetcher_.get() != nullptr)
        << ": No status";

    constexpr double kEpsTheta = 0.01;
    constexpr double kEpsOmega = 0.01;

    // Check that the status had the right goal
    ASSERT_NEAR(points_[superstructure_goal_fetcher_->arm_goal_position()](0),
                superstructure_status_fetcher_->arm()->goal_theta0(),
                kEpsTheta);
    ASSERT_NEAR(points_[superstructure_goal_fetcher_->arm_goal_position()](1),
                superstructure_status_fetcher_->arm()->goal_theta1(),
                kEpsTheta);
    ASSERT_NEAR(points_[superstructure_goal_fetcher_->arm_goal_position()](2),
                superstructure_status_fetcher_->arm()->goal_theta2(),
                kEpsTheta);

    // Check that the status met the goal
    EXPECT_NEAR(superstructure_status_fetcher_->arm()->goal_theta0(),
                superstructure_status_fetcher_->arm()->theta0(), kEpsTheta);
    EXPECT_NEAR(superstructure_status_fetcher_->arm()->goal_theta1(),
                superstructure_status_fetcher_->arm()->theta1(), kEpsTheta);
    EXPECT_NEAR(superstructure_status_fetcher_->arm()->goal_theta2(),
                superstructure_status_fetcher_->arm()->theta2(), kEpsTheta);
    EXPECT_NEAR(superstructure_status_fetcher_->arm()->goal_omega0(),
                superstructure_status_fetcher_->arm()->omega0(), kEpsOmega);
    EXPECT_NEAR(superstructure_status_fetcher_->arm()->goal_omega1(),
                superstructure_status_fetcher_->arm()->omega1(), kEpsOmega);
    EXPECT_NEAR(superstructure_status_fetcher_->arm()->goal_omega2(),
                superstructure_status_fetcher_->arm()->omega2(), kEpsOmega);

    // Check that our simulator matches the status
    EXPECT_NEAR(superstructure_plant_.arm()->proximal_position(),
                superstructure_status_fetcher_->arm()->theta0(), kEpsTheta);
    EXPECT_NEAR(superstructure_plant_.arm()->distal_position(),
                superstructure_status_fetcher_->arm()->theta1(), kEpsTheta);
    EXPECT_NEAR(superstructure_plant_.arm()->roll_joint_position(),
                superstructure_status_fetcher_->arm()->theta2(), kEpsTheta);
    EXPECT_NEAR(superstructure_plant_.arm()->proximal_velocity(),
                superstructure_status_fetcher_->arm()->omega0(), kEpsOmega);
    EXPECT_NEAR(superstructure_plant_.arm()->distal_velocity(),
                superstructure_status_fetcher_->arm()->omega1(), kEpsOmega);
    EXPECT_NEAR(superstructure_plant_.arm()->roll_joint_velocity(),
                superstructure_status_fetcher_->arm()->omega2(), kEpsOmega);

    if (superstructure_goal_fetcher_->has_wrist()) {
      EXPECT_NEAR(superstructure_goal_fetcher_->wrist()->unsafe_goal(),
                  superstructure_status_fetcher_->wrist()->position(), 0.001);
    }
  }

  void CheckIfZeroed() {
    superstructure_status_fetcher_.Fetch();
    ASSERT_TRUE(superstructure_status_fetcher_.get()->zeroed());
  }

  void WaitUntilZeroed() {
    int i = 0;
    do {
      i++;
      RunFor(dt());
      superstructure_status_fetcher_.Fetch();
      // 2 Seconds
      ASSERT_LE(i, 2.0 / ::aos::time::DurationInSeconds(dt()));

      // Since there is a delay when sending running, make sure we have a
      // status before checking it.
    } while (superstructure_status_fetcher_.get() == nullptr ||
             !superstructure_status_fetcher_.get()->zeroed());

    superstructure_plant_.wrist()->set_voltage_offset(1.0);
  }

  void SendRobotVelocity(double robot_velocity) {
    SendDrivetrainStatus(robot_velocity, {0.0, 0.0}, 0.0);
  }

  void SendDrivetrainStatus(double robot_velocity, Eigen::Vector2d pos,
                            double theta) {
    // Send a robot velocity to test compensation
    auto builder = drivetrain_status_sender_.MakeBuilder();
    auto drivetrain_status_builder = builder.MakeBuilder<DrivetrainStatus>();
    drivetrain_status_builder.add_robot_speed(robot_velocity);
    drivetrain_status_builder.add_estimated_left_velocity(robot_velocity);
    drivetrain_status_builder.add_estimated_right_velocity(robot_velocity);
    drivetrain_status_builder.add_x(pos.x());
    drivetrain_status_builder.add_y(pos.y());
    drivetrain_status_builder.add_theta(theta);
    builder.CheckOk(builder.Send(drivetrain_status_builder.Finish()));
  }

  std::shared_ptr<const constants::Values> values_;

  const aos::Node *const roborio_;
  const aos::Node *const logger_pi_;

  const ::aos::FlatbufferVector<ArmTrajectories> arm_trajectories_;

  ::std::unique_ptr<::aos::EventLoop> superstructure_event_loop;
  ::y2023::control_loops::superstructure::Superstructure superstructure_;
  ::std::unique_ptr<::aos::EventLoop> test_event_loop_;
  ::aos::PhasedLoopHandler *phased_loop_handle_ = nullptr;

  ::aos::Fetcher<Goal> superstructure_goal_fetcher_;
  ::aos::Sender<Goal> superstructure_goal_sender_;
  ::aos::Fetcher<Status> superstructure_status_fetcher_;
  ::aos::Fetcher<Output> superstructure_output_fetcher_;
  ::aos::Fetcher<Position> superstructure_position_fetcher_;
  ::aos::Sender<Position> superstructure_position_sender_;
  ::aos::Sender<DrivetrainStatus> drivetrain_status_sender_;

  ::std::unique_ptr<::aos::EventLoop> superstructure_plant_event_loop_;
  SuperstructureSimulation superstructure_plant_;

  std::unique_ptr<aos::EventLoop> logger_event_loop_;
  std::unique_ptr<aos::logger::Logger> logger_;

  const ::std::vector<::Eigen::Matrix<double, 3, 1>> points_;
};

// Tests that the superstructure does nothing when the goal is to remain
// still.
TEST_F(SuperstructureTest, DoesNothing) {
  SetEnabled(true);
  WaitUntilZeroed();

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        wrist_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kCompWristRange().middle());

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_arm_goal_position(arm::NeutralIndex());
    goal_builder.add_trajectory_override(false);
    goal_builder.add_wrist(wrist_offset);
    goal_builder.add_roller_goal(RollerGoal::IDLE);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }
  RunFor(chrono::seconds(10));
  VerifyNearGoal();

  EXPECT_TRUE(superstructure_output_fetcher_.Fetch());
}

// Tests that loops can reach a goal.
TEST_F(SuperstructureTest, ReachesGoal) {
  SetEnabled(true);
  WaitUntilZeroed();
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        wrist_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kCompWristRange().upper);

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_arm_goal_position(arm::NeutralIndex());
    goal_builder.add_trajectory_override(false);
    goal_builder.add_wrist(wrist_offset);
    goal_builder.add_roller_goal(RollerGoal::IDLE);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  // Give it a lot of time to get there.
  RunFor(chrono::seconds(15));

  VerifyNearGoal();
}

// Makes sure that the voltage on a motor is properly pulled back after
// saturation such that we don't get weird or bad (e.g. oscillating)
// behaviour.
TEST_F(SuperstructureTest, SaturationTest) {
  SetEnabled(true);

  // Zero it before we move.
  WaitUntilZeroed();
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        wrist_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kCompWristRange().upper);

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_wrist(wrist_offset);
    goal_builder.add_arm_goal_position(arm::NeutralIndex());

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }
  RunFor(chrono::seconds(20));
  VerifyNearGoal();

  // Try a low acceleration move with a high max velocity and verify the
  // acceleration is capped like expected.
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        wrist_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), constants::Values::kCompWristRange().lower,
            CreateProfileParameters(*builder.fbb(), 20.0, 0.1));

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_wrist(wrist_offset);

    goal_builder.add_arm_goal_position(arm::NeutralIndex());
    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  superstructure_plant_.wrist()->set_peak_velocity(23.0);
  superstructure_plant_.wrist()->set_peak_acceleration(0.2);

  // TODO(Milo): Make this a sane time
  RunFor(chrono::seconds(20));
  VerifyNearGoal();
}

// Tests that the loop zeroes when run for a while without a goal.
TEST_F(SuperstructureTest, ZeroNoGoal) {
  SetEnabled(true);
  WaitUntilZeroed();
  RunFor(chrono::seconds(2));

  EXPECT_EQ(ArmState::RUNNING, superstructure_.arm().state());
  EXPECT_EQ(Superstructure::AbsoluteEncoderSubsystem::State::RUNNING,
            superstructure_.wrist().state());
}

// Tests that running disabled works
TEST_F(SuperstructureTest, DisableTest) {
  RunFor(chrono::seconds(2));
  CheckIfZeroed();
}

class SuperstructureBeambreakTest
    : public SuperstructureTest,
      public ::testing::WithParamInterface<GamePiece> {
 public:
  void SetBeambreak(GamePiece game_piece, bool status) {
    if (game_piece == GamePiece::CONE) {
      // TODO(milind): handle cone
    } else {
      superstructure_plant_.set_end_effector_cube_beam_break(status);
    }
  }
};

TEST_P(SuperstructureBeambreakTest, EndEffectorGoal) {
  SetEnabled(true);
  WaitUntilZeroed();

  double spit_voltage =
      (GetParam() == GamePiece::CUBE ? EndEffector::kRollerCubeSpitVoltage()
                                     : EndEffector::kRollerConeSpitVoltage());
  double suck_voltage =
      (GetParam() == GamePiece::CUBE ? EndEffector::kRollerCubeSuckVoltage()
                                     : EndEffector::kRollerConeSuckVoltage());

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_arm_goal_position(arm::NeutralIndex());
    goal_builder.add_trajectory_override(false);
    goal_builder.add_roller_goal(GetParam() == GamePiece::CONE
                                     ? RollerGoal::INTAKE_CONE
                                     : RollerGoal::INTAKE_CUBE);

    builder.CheckOk(builder.Send(goal_builder.Finish()));
  }
  SetBeambreak(GetParam(), false);

  // This makes sure that we intake as normal when
  // requesting intake.
  RunFor(constants::Values::kExtraIntakingTime());

  ASSERT_TRUE(superstructure_output_fetcher_.Fetch());
  ASSERT_TRUE(superstructure_status_fetcher_.Fetch());

  EXPECT_EQ(superstructure_output_fetcher_->roller_voltage(), suck_voltage);
  EXPECT_EQ(superstructure_status_fetcher_->end_effector_state(),
            EndEffectorState::INTAKING);
  EXPECT_EQ(superstructure_status_fetcher_->game_piece(), GetParam());

  SetBeambreak(GetParam(), true);

  // Checking that after the beambreak is set once intaking that the
  // state changes to LOADED.
  RunFor(dt());

  ASSERT_TRUE(superstructure_output_fetcher_.Fetch());
  ASSERT_TRUE(superstructure_status_fetcher_.Fetch());

  EXPECT_EQ(superstructure_output_fetcher_->roller_voltage(), 0.0);
  EXPECT_EQ(superstructure_status_fetcher_->end_effector_state(),
            EndEffectorState::LOADED);
  EXPECT_EQ(superstructure_status_fetcher_->game_piece(), GetParam());

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_arm_goal_position(arm::NeutralIndex());
    goal_builder.add_trajectory_override(false);
    goal_builder.add_roller_goal(RollerGoal::IDLE);

    builder.CheckOk(builder.Send(goal_builder.Finish()));
  }

  SetBeambreak(GetParam(), false);
  // Checking that it's going back to intaking because we lost the
  // beambreak sensor.
  RunFor(dt() * 2);

  ASSERT_TRUE(superstructure_output_fetcher_.Fetch());
  ASSERT_TRUE(superstructure_status_fetcher_.Fetch());

  EXPECT_EQ(superstructure_output_fetcher_->roller_voltage(), suck_voltage);
  EXPECT_EQ(superstructure_status_fetcher_->end_effector_state(),
            EndEffectorState::INTAKING);
  EXPECT_EQ(superstructure_status_fetcher_->game_piece(), GetParam());

  // Checking that we go back to idle after beambreak is lost and we
  // set our goal to idle.
  RunFor(dt() * 2 + constants::Values::kExtraIntakingTime());
  ASSERT_TRUE(superstructure_output_fetcher_.Fetch());
  ASSERT_TRUE(superstructure_status_fetcher_.Fetch());

  EXPECT_EQ(superstructure_output_fetcher_->roller_voltage(), 0.0);
  EXPECT_EQ(superstructure_status_fetcher_->end_effector_state(),
            EndEffectorState::IDLE);
  EXPECT_EQ(superstructure_status_fetcher_->game_piece(), GetParam());

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_arm_goal_position(arm::NeutralIndex());
    goal_builder.add_trajectory_override(false);
    goal_builder.add_roller_goal(GetParam() == GamePiece::CONE
                                     ? RollerGoal::INTAKE_CONE
                                     : RollerGoal::INTAKE_CUBE);

    builder.CheckOk(builder.Send(goal_builder.Finish()));
  }

  // Going through intake -> loaded -> spitting
  // Making sure that it's intaking normally.
  RunFor(constants::Values::kExtraIntakingTime());

  ASSERT_TRUE(superstructure_output_fetcher_.Fetch());
  ASSERT_TRUE(superstructure_status_fetcher_.Fetch());

  EXPECT_EQ(superstructure_output_fetcher_->roller_voltage(), suck_voltage);
  EXPECT_EQ(superstructure_status_fetcher_->end_effector_state(),
            EndEffectorState::INTAKING);
  EXPECT_EQ(superstructure_status_fetcher_->game_piece(), GetParam());

  SetBeambreak(GetParam(), true);

  // Checking that it's loaded once beambreak is sensing something.
  RunFor(dt());

  ASSERT_TRUE(superstructure_output_fetcher_.Fetch());
  ASSERT_TRUE(superstructure_status_fetcher_.Fetch());

  EXPECT_EQ(superstructure_output_fetcher_->roller_voltage(), 0.0);
  EXPECT_EQ(superstructure_status_fetcher_->end_effector_state(),
            EndEffectorState::LOADED);
  EXPECT_EQ(superstructure_status_fetcher_->game_piece(), GetParam());

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_arm_goal_position(arm::NeutralIndex());
    goal_builder.add_trajectory_override(false);
    goal_builder.add_roller_goal(RollerGoal::SPIT);

    builder.CheckOk(builder.Send(goal_builder.Finish()));
  }

  // Checking that it stays spitting until 2 seconds after the
  // beambreak is lost.
  RunFor(dt() * 10);

  ASSERT_TRUE(superstructure_output_fetcher_.Fetch());
  ASSERT_TRUE(superstructure_status_fetcher_.Fetch());

  EXPECT_EQ(superstructure_output_fetcher_->roller_voltage(), spit_voltage);
  EXPECT_EQ(superstructure_status_fetcher_->end_effector_state(),
            EndEffectorState::SPITTING);
  EXPECT_EQ(superstructure_status_fetcher_->game_piece(), GetParam());

  SetBeambreak(GetParam(), false);

  RunFor(constants::Values::kExtraSpittingTime());

  ASSERT_TRUE(superstructure_output_fetcher_.Fetch());
  ASSERT_TRUE(superstructure_status_fetcher_.Fetch());

  EXPECT_EQ(superstructure_output_fetcher_->roller_voltage(), spit_voltage);
  EXPECT_EQ(superstructure_status_fetcher_->end_effector_state(),
            EndEffectorState::SPITTING);
  EXPECT_EQ(superstructure_status_fetcher_->game_piece(), GetParam());

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();

    goal_builder.add_arm_goal_position(arm::NeutralIndex());
    goal_builder.add_trajectory_override(false);
    goal_builder.add_roller_goal(RollerGoal::IDLE);

    builder.CheckOk(builder.Send(goal_builder.Finish()));
  }

  // Checking that it goes to idle after it's given time to stop spitting.
  RunFor(dt() * 3);

  ASSERT_TRUE(superstructure_output_fetcher_.Fetch());
  ASSERT_TRUE(superstructure_status_fetcher_.Fetch());

  EXPECT_EQ(superstructure_output_fetcher_->roller_voltage(), 0.0);
  EXPECT_EQ(superstructure_status_fetcher_->end_effector_state(),
            EndEffectorState::IDLE);
  EXPECT_EQ(superstructure_status_fetcher_->game_piece(), GamePiece::NONE);
}

// Tests that we don't freak out without a goal.
TEST_F(SuperstructureTest, ArmSimpleGoal) {
  SetEnabled(true);
  WaitUntilZeroed();
  RunFor(chrono::seconds(20));

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_arm_goal_position(arm::ScoreBackMidConeUpIndex());
    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  ASSERT_TRUE(superstructure_status_fetcher_.Fetch());
  EXPECT_EQ(ArmState::RUNNING, superstructure_status_fetcher_->arm()->state());
}

// Tests that we can can execute a move.
TEST_F(SuperstructureTest, ArmMoveAndMoveBack) {
  SetEnabled(true);
  WaitUntilZeroed();
  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_arm_goal_position(arm::NeutralIndex());
    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  RunFor(chrono::seconds(10));

  VerifyNearGoal();

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_arm_goal_position(arm::ScoreBackMidConeUpIndex());
    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  RunFor(chrono::seconds(10));
  VerifyNearGoal();
}

// Tests that we can can execute a move which moves through multiple nodes.
TEST_F(SuperstructureTest, ArmMultistepMove) {
  SetEnabled(true);
  WaitUntilZeroed();
  superstructure_plant_.InitializeArmPosition(arm::NeutralPoint());

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_arm_goal_position(arm::ScoreBackMidConeUpIndex());
    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  RunFor(chrono::seconds(10));

  VerifyNearGoal();

  {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_arm_goal_position(arm::ScoreFrontLowConeUpIndex());
    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  RunFor(chrono::seconds(10));
  VerifyNearGoal();
}

// TODO(milind): add cone
INSTANTIATE_TEST_SUITE_P(EndEffectorGoal, SuperstructureBeambreakTest,
                         ::testing::Values(GamePiece::CUBE));

}  // namespace testing
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2023
