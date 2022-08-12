#include <chrono>
#include <memory>

#include "aos/events/logging/log_writer.h"
#include "frc971/control_loops/capped_test_plant.h"
#include "frc971/control_loops/control_loop_test.h"
#include "frc971/control_loops/position_sensor_sim.h"
#include "frc971/control_loops/team_number_test_environment.h"
#include "gtest/gtest.h"
#include "y2022_bot3/control_loops/drivetrain/drivetrain_dog_motor_plant.h"
#include "y2022_bot3/control_loops/superstructure/superstructure.h"

DEFINE_string(output_folder, "",
              "If set, logs all channels to the provided logfile.");

namespace y2022_bot3 {
namespace control_loops {
namespace superstructure {
namespace testing {
namespace chrono = std::chrono;
namespace {
constexpr double kNoiseScalar = 0.01;
}

using ::aos::monotonic_clock;
using ::frc971::CreateProfileParameters;
using ::frc971::control_loops::CappedTestPlant;
using ::frc971::control_loops::
    CreateStaticZeroingSingleDOFProfiledSubsystemGoal;
using ::frc971::control_loops::PositionSensorSimulator;
using ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoal;
typedef Superstructure::PotAndAbsoluteEncoderSubsystem
    PotAndAbsoluteEncoderSubsystem;
using DrivetrainStatus = ::frc971::control_loops::drivetrain::Status;

template <typename SubsystemStatus, typename SubsystemState,
          typename SubsystemConstants>

// TODO(Henry) put this in frc971
class SubsystemSimulator {
 public:
  SubsystemSimulator(CappedTestPlant *plant, PositionSensorSimulator encoder,
                     const SubsystemConstants subsystem_constants,
                     const frc971::constants::Range range,
                     double encoder_offset, const chrono::nanoseconds dt)
      : plant_(plant),
        encoder_(encoder),
        subsystem_constants_(subsystem_constants),
        range_(range),
        encoder_offset_(encoder_offset),
        dt_(dt) {}

  void InitializePosition(double start_pos) {
    plant_->mutable_X(0, 0) = start_pos;
    plant_->mutable_X(1, 0) = 0.0;

    encoder_.Initialize(start_pos, kNoiseScalar, 0.0, encoder_offset_);
  }

  // Simulates the superstructure for a single timestep.
  void Simulate(double voltage, const SubsystemStatus *status) {
    double last_velocity = plant_->X(1, 0);

    const double voltage_check =
        (static_cast<SubsystemState>(status->state()) ==
         SubsystemState::RUNNING)
            ? subsystem_constants_.subsystem_params.operating_voltage
            : subsystem_constants_.subsystem_params.zeroing_voltage;

    EXPECT_NEAR(voltage, 0.0, voltage_check);

    ::Eigen::Matrix<double, 1, 1> U;
    U << voltage + plant_->voltage_offset();
    plant_->Update(U);

    const double position = plant_->Y(0, 0);

    encoder_.MoveTo(position);

    EXPECT_GE(position, range_.lower_hard);
    EXPECT_LE(position, range_.upper_hard);

    const double loop_time = ::aos::time::DurationInSeconds(dt_);

    const double velocity = plant_->X(1, 0);
    const double acceleration = (velocity - last_velocity) / loop_time;

    EXPECT_GE(peak_acceleration_, acceleration);
    EXPECT_LE(-peak_acceleration_, acceleration);
    EXPECT_GE(peak_velocity_, velocity);
    EXPECT_LE(-peak_velocity_, velocity);
  }

  void set_peak_acceleration(double value) { peak_acceleration_ = value; }
  void set_peak_velocity(double value) { peak_velocity_ = value; }

  void set_controller_index(size_t index) { plant_->set_index(index); }

  PositionSensorSimulator *encoder() { return &encoder_; }

 private:
  std::unique_ptr<CappedTestPlant> plant_;
  PositionSensorSimulator encoder_;
  const SubsystemConstants subsystem_constants_;
  const frc971::constants::Range range_;

  double encoder_offset_ = 0.0;

  double peak_velocity_ = std::numeric_limits<double>::infinity();
  double peak_acceleration_ = std::numeric_limits<double>::infinity();

  const chrono::nanoseconds dt_;
};

using PotAndAbsoluteEncoderSimulator = SubsystemSimulator<
    frc971::control_loops::PotAndAbsoluteEncoderProfiledJointStatus,
    PotAndAbsoluteEncoderSubsystem::State,
    constants::Values::PotAndAbsEncoderConstants>;

// Class which simulates the superstructure and sends out queue messages with
// the position.
class SuperstructureSimulation {
 public:
  SuperstructureSimulation(::aos::EventLoop *event_loop,
                           std::shared_ptr<const constants::Values> values,
                           chrono::nanoseconds dt)
      : event_loop_(event_loop),
        dt_(dt),
        superstructure_position_sender_(
            event_loop_->MakeSender<Position>("/superstructure")),
        superstructure_status_fetcher_(
            event_loop_->MakeFetcher<Status>("/superstructure")),
        superstructure_output_fetcher_(
            event_loop_->MakeFetcher<Output>("/superstructure")) {
    phased_loop_handle_ = event_loop_->AddPhasedLoop(
        [this](int) {
          // Skip this the first time.
          if (!first_) {
            EXPECT_TRUE(superstructure_output_fetcher_.Fetch());
            EXPECT_TRUE(superstructure_status_fetcher_.Fetch());
          }
          first_ = false;
          SendPositionMessage();
        },
        dt);
    (void)values;
    (void)dt_;
  }

  // Sends a queue message with the position of the superstructure.
  void SendPositionMessage() {
    ::aos::Sender<Position>::Builder builder =
        superstructure_position_sender_.MakeBuilder();
  }

 private:
  ::aos::EventLoop *event_loop_;
  const chrono::nanoseconds dt_;
  ::aos::PhasedLoopHandler *phased_loop_handle_ = nullptr;

  ::aos::Sender<Position> superstructure_position_sender_;
  ::aos::Fetcher<Status> superstructure_status_fetcher_;
  ::aos::Fetcher<Output> superstructure_output_fetcher_;

  bool first_ = true;
};

class SuperstructureTest : public ::frc971::testing::ControlLoopTest {
 public:
  SuperstructureTest()
      : ::frc971::testing::ControlLoopTest(
            aos::configuration::ReadConfig("y2022_bot3/aos_config.json"),
            std::chrono::microseconds(5050)),
        values_(std::make_shared<constants::Values>(constants::MakeValues(
            frc971::control_loops::testing::kTeamNumber))),
        roborio_(aos::configuration::GetNode(configuration(), "roborio")),
        superstructure_event_loop(MakeEventLoop("Superstructure", roborio_)),
        superstructure_(superstructure_event_loop.get(), values_),
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
                              dt()) {
    set_team_id(frc971::control_loops::testing::kTeamNumber);

    SetEnabled(true);
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

  ::std::unique_ptr<::aos::EventLoop> superstructure_event_loop;
  ::y2022_bot3::control_loops::superstructure::Superstructure superstructure_;
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

};  // namespace testing

}  // namespace testing
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2022_bot3
