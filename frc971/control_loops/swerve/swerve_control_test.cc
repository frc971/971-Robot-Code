#include <unistd.h>

#include <chrono>
#include <memory>
#include <vector>

#include "absl/strings/str_format.h"
#include "gtest/gtest.h"

#include "aos/events/shm_event_loop.h"
#include "frc971/control_loops/control_loop_test.h"
#include "frc971/control_loops/swerve/swerve_control_loops.h"
#include "frc971/control_loops/swerve/swerve_drivetrain_can_position_static.h"
#include "frc971/control_loops/swerve/swerve_drivetrain_goal_static.h"
#include "frc971/control_loops/swerve/swerve_drivetrain_output_generated.h"
#include "frc971/control_loops/swerve/swerve_drivetrain_position_static.h"
#include "frc971/control_loops/swerve/swerve_drivetrain_status_generated.h"
#include "frc971/control_loops/team_number_test_environment.h"
#include "frc971/queues/gyro_static.h"

using namespace std;

namespace frc971::control_loops::swerve::testing {
namespace chrono = ::std::chrono;
using aos::monotonic_clock;
namespace {
template <typename Scalar>
frc971::control_loops::swerve::SimplifiedDynamics<Scalar>::Parameters
MakeSwerveParameters() {
  auto make_module = [](const Eigen::Matrix<Scalar, 2, 1> &position) {
    return frc971::control_loops::swerve::LinearVelocityController::Dynamics::
        ModuleParams{.position = position,
                     .slip_angle_coefficient = 0.0,
                     .slip_angle_alignment_coefficient = 0.0,
                     .steer_motor = frc971::control_loops::swerve::KrakenFOC(),
                     .drive_motor = frc971::control_loops::swerve::KrakenFOC(),
                     .steer_ratio = (1.0 / 12.1),
                     .drive_ratio = (12.0 / 54.0 * 38.0 / 16.0 * 15.0 / 45.0) *
                                    1.8 * 0.0254,
                     .wheel_radius = 1.8 * 0.0254,
                     .extra_steer_inertia = 0.01};
  };

  constexpr Scalar kSideLength = 0.635;
  return {.mass = 35,
          .moment_of_inertia = 2.63,
          .modules =
              {
                  // front left
                  make_module({kSideLength / 2.0, kSideLength / 2.0}),
                  // front right
                  make_module({kSideLength / 2.0, -kSideLength / 2.0}),
                  // back left
                  make_module({-kSideLength / 2.0, kSideLength / 2.0}),
                  // back right
                  make_module({-kSideLength / 2.0, -kSideLength / 2.0}),
              },
          .accel_weight = 0.0};
}
}  // namespace

// Class which simulates stuff and sends out queue messages with the position.
class SwerveControlSimulation {
 public:
  SwerveControlSimulation(::aos::EventLoop *event_loop, chrono::nanoseconds dt)
      : event_loop_(event_loop),
        position_sender_(event_loop_->MakeSender<PositionStatic>("/swerve")),
        can_position_sender_(
            event_loop_->MakeSender<CanPositionStatic>("/swerve")),
        goal_fetcher_(event_loop_->MakeFetcher<Goal>("/swerve")),
        status_fetcher_(event_loop_->MakeFetcher<Status>("/swerve")),
        output_fetcher_(event_loop_->MakeFetcher<Output>("/swerve")),
        gyro_velocity_(0.0) {
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
    {
      auto builder = position_sender_.MakeStaticBuilder();

      builder->add_front_left()->add_rotation_position()->set_encoder(
          module_positions_(0, 0));
      builder->add_front_right()->add_rotation_position()->set_encoder(
          module_positions_(1, 0));
      builder->add_back_left()->add_rotation_position()->set_encoder(
          module_positions_(2, 0));
      builder->add_back_right()->add_rotation_position()->set_encoder(
          module_positions_(3, 0));

      EXPECT_EQ(builder.Send(), aos::RawSender::Error::kOk);
    }

    // TODO(max): Find a way to send goal_velocity_ without buildkite crashing
    {
      auto builder = can_position_sender_.MakeStaticBuilder();

      int i = 0;
      for (auto &module :
           {builder->add_front_left(), builder->add_front_right(),
            builder->add_back_left(), builder->add_back_right()}) {
        module->add_rotation()->set_position(module_positions_(i, 0));

        auto translation = module->add_translation();

        translation->set_position(module_positions_(i, 1));
        translation->set_timestamp(
            std::chrono::time_point_cast<std::chrono::nanoseconds>(
                event_loop_->monotonic_now())
                .time_since_epoch()
                .count());

        i++;
      }

      builder.CheckOk(builder.Send());
    }
  }

  void VerifyNearGoal() {
    goal_fetcher_.Fetch();
    status_fetcher_.Fetch();

    ASSERT_TRUE(goal_fetcher_.get() != nullptr) << ": No goal";
    ASSERT_TRUE(status_fetcher_.get() != nullptr) << ": No status";

    constexpr double kEpsTheta = 0.03;
    constexpr double kEpsRotationVelocity = 0.03;
    constexpr double kEpsTranslationSpeed = 0.03;

    EXPECT_NEAR(status_fetcher_->naive_estimator()->x_hat()->data()->Get(
                    SimplifiedDynamics<double>::States::kVx),
                goal_fetcher_->joystick_goal()->vx(), kEpsTranslationSpeed);

    EXPECT_NEAR(status_fetcher_->naive_estimator()->x_hat()->data()->Get(
                    SimplifiedDynamics<double>::States::kVy),
                goal_fetcher_->joystick_goal()->vy(), kEpsTranslationSpeed);

    EXPECT_NEAR(status_fetcher_->naive_estimator()->x_hat()->data()->Get(
                    SimplifiedDynamics<double>::States::kOmega),
                goal_fetcher_->joystick_goal()->omega(), kEpsRotationVelocity);

    for (auto &[goal_angle, state] : {
             std::pair<double, double>{
                 goal_fetcher_->front_left_goal()->rotation_angle(),
                 SimplifiedDynamics<double>::States::kThetas0},
             std::pair<double, double>{
                 goal_fetcher_->front_right_goal()->rotation_angle(),
                 SimplifiedDynamics<double>::States::kThetas1},
             std::pair<double, double>{
                 goal_fetcher_->back_left_goal()->rotation_angle(),
                 SimplifiedDynamics<double>::States::kThetas2},
             std::pair<double, double>{
                 goal_fetcher_->back_right_goal()->rotation_angle(),
                 SimplifiedDynamics<double>::States::kThetas3},
         }) {
      EXPECT_NEAR(
          status_fetcher_->naive_estimator()->x_hat()->data()->Get(state),
          goal_angle, kEpsTheta);
    }
  }

  // Simulates basic control loop for a single timestep.
  void Simulate() { EXPECT_TRUE(output_fetcher_.Fetch()); }

  void set_gyro_velocity(double velocity) { gyro_velocity_ = velocity; }

  void set_module_positions(Eigen::Matrix<double, 4, 2> positions) {
    module_positions_ = positions;
  }

 private:
  ::aos::EventLoop *event_loop_;

  ::aos::Sender<PositionStatic> position_sender_;
  ::aos::Sender<CanPositionStatic> can_position_sender_;
  ::aos::Sender<Goal> goal_sender_;

  ::aos::Fetcher<CanPosition> can_position_fetcher_;
  ::aos::Fetcher<Position> position_fetcher_;
  ::aos::Fetcher<Goal> goal_fetcher_;
  ::aos::Fetcher<Status> status_fetcher_;
  ::aos::Fetcher<Output> output_fetcher_;

  double gyro_velocity_;

  Eigen::Matrix<double, 4, 2> module_positions_;

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
        subsystem_params_(
            aos::JsonToFlatbuffer<
                frc971::control_loops::
                    StaticZeroingSingleDOFProfiledSubsystemCommonParams>(
                absl::StrFormat(R"json({
                    "zeroing_voltage": 3.0,
                    "operating_voltage": 12.0,
                    "zeroing_profile_params": {
                      "max_velocity": 0.5,
                      "max_acceleration": 3.0
                    },
                    "default_profile_params":{
                      "max_velocity": 12.0,
                      "max_acceleration": 55.0
                    },
                    "range": {
                        "lower_hard": -inf,
                        "upper_hard": inf,
                        "lower": -inf,
                        "upper": inf
                    },
                    "loop": %s
                    })json",
                                aos::util::ReadFileToStringOrDie(
                                    "frc971/control_loops/swerve/test_module/"
                                    "integral_rotation_plant.json")))),
        zeroing_params_(aos::JsonToFlatbuffer<SwerveZeroing>(R"json({
        "front_left": {
          "average_filter_size": 10,
          "one_revolution_distance": 6,
          "moving_buffer_size": 10
        },
        "front_right": {
          "average_filter_size": 10,
          "one_revolution_distance": 6,
          "moving_buffer_size": 10
        },
        "back_left": {
          "average_filter_size": 10,
          "one_revolution_distance": 6,
          "moving_buffer_size": 10
        },
        "back_right": {
          "average_filter_size": 10,
          "one_revolution_distance": 6,
          "moving_buffer_size": 10
        }
        })json")),
        swerve_control_loops_(
            swerve_control_event_loop_.get(), &subsystem_params_.message(),
            &zeroing_params_.message(), MakeSwerveParameters<float>(),
            LinearVelocityController::ControllerWeights{
                .thetas_q = 1.0,
                .omegas_q = 1e-4,
                .vel_q = 50.0,
                .theta_q = 50000.0,
                .omega_q = 7.0,
                .steer_current_r = 1e-5,
                .drive_current_r = 1e-3,
            },
            "/swerve"),

        swerve_control_simulation_event_loop_(MakeEventLoop("simulation")),
        swerve_control_simulation_(swerve_control_simulation_event_loop_.get(),
                                   dt()) {
    set_team_id(control_loops::testing::kTeamNumber);
    SetEnabled(true);
  }

  ::std::unique_ptr<::aos::EventLoop> swerve_test_event_loop_;
  ::aos::Sender<Goal> goal_sender_;

  ::std::unique_ptr<::aos::EventLoop> swerve_control_event_loop_;
  aos::FlatbufferDetachedBuffer<
      frc971::control_loops::
          StaticZeroingSingleDOFProfiledSubsystemCommonParams>
      subsystem_params_;
  aos::FlatbufferDetachedBuffer<SwerveZeroing> zeroing_params_;
  SwerveControlLoops swerve_control_loops_;

  ::std::unique_ptr<::aos::EventLoop> swerve_control_simulation_event_loop_;
  SwerveControlSimulation swerve_control_simulation_;
};

// Tests that the swerve modules' speeds are all set to 0.
TEST_F(SwerveControlLoopTest, SwerveModulesDontMove) {
  {
    auto builder = goal_sender_.MakeBuilder();

    JoystickGoal::Builder joystick_goal_builder =
        builder.MakeBuilder<JoystickGoal>();
    joystick_goal_builder.add_vx(0.0);
    joystick_goal_builder.add_vy(0.0);
    joystick_goal_builder.add_omega(0.0);
    joystick_goal_builder.add_auto_align(false);
    auto joystick_goal_offset = joystick_goal_builder.Finish();

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
    goal_builder.add_joystick_goal(joystick_goal_offset);

    ASSERT_EQ(builder.Send(goal_builder.Finish()), aos::RawSender::Error::kOk);
  }

  swerve_control_simulation_.set_module_positions(
      Eigen::Matrix<double, 4, 2>::Zero());
  swerve_control_simulation_.set_gyro_velocity(0.0);

  RunFor(dt() * 2);

  swerve_control_simulation_.VerifyNearGoal();
}

}  // namespace frc971::control_loops::swerve::testing
