#include <unistd.h>

#include <cmath>
#include <cstdio>
#include <cstring>

#include "aos/actions/actions.h"
#include "aos/init.h"
#include "aos/logging/logging.h"
#include "aos/network/team_number.h"
#include "aos/util/log_interval.h"
#include "frc971/autonomous/base_autonomous_actor.h"
#include "frc971/control_loops/drivetrain/localizer_generated.h"
#include "frc971/control_loops/profiled_subsystem_generated.h"
#include "frc971/input/action_joystick_input.h"
#include "frc971/input/driver_station_data.h"
#include "frc971/input/drivetrain_input.h"
#include "frc971/input/joystick_input.h"
#include "frc971/zeroing/wrap.h"
#include "y2022/constants.h"
#include "y2022/control_loops/drivetrain/drivetrain_base.h"
#include "y2022/control_loops/superstructure/superstructure_goal_generated.h"
#include "y2022/control_loops/superstructure/superstructure_status_generated.h"
#include "y2022/setpoint_generated.h"

using frc971::CreateProfileParameters;
using frc971::control_loops::CreateStaticZeroingSingleDOFProfiledSubsystemGoal;
using frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoal;
using frc971::input::driver_station::ButtonLocation;
using frc971::input::driver_station::ControlBit;
using frc971::input::driver_station::JoystickAxis;
using frc971::input::driver_station::POVLocation;

namespace y2022 {
namespace input {
namespace joysticks {

namespace superstructure = y2022::control_loops::superstructure;

#if 0
const ButtonLocation kCatapultPos(4, 3);
const ButtonLocation kFire(3, 4);
const ButtonLocation kTurret(4, 15);
const ButtonLocation kAutoAim(4, 2);

const ButtonLocation kIntakeFrontOut(4, 10);
const ButtonLocation kIntakeBackOut(4, 9);
const ButtonLocation kSpitFront(3, 3);
const ButtonLocation kSpitBack(2, 3);

const ButtonLocation kRedLocalizerReset(3, 13);
const ButtonLocation kBlueLocalizerReset(3, 14);
const ButtonLocation kLocalizerReset(3, 8);
#else

const ButtonLocation kCatapultPos(4, 3);
const ButtonLocation kFire(4, 1);
const ButtonLocation kTurret(4, 15);
const ButtonLocation kAutoAim(4, 16);

const ButtonLocation kClimberExtend(1, 2);
const ButtonLocation kClimberIntakes(4, 5);
const ButtonLocation kClimberServo(4, 4);

const ButtonLocation kIntakeFrontOut(4, 10);
const ButtonLocation kIntakeBackOut(4, 9);
const ButtonLocation kSpitFront(4, 8);
const ButtonLocation kSpitBack(4, 7);
const ButtonLocation kSpit(3, 3);

const ButtonLocation kRedLocalizerReset(4, 14);
const ButtonLocation kBlueLocalizerReset(4, 13);
const ButtonLocation kLocalizerReset(3, 8);
#endif

class Reader : public ::frc971::input::ActionJoystickInput {
 public:
  Reader(::aos::EventLoop *event_loop)
      : ::frc971::input::ActionJoystickInput(
            event_loop,
            ::y2022::control_loops::drivetrain::GetDrivetrainConfig(),
            ::frc971::input::DrivetrainInputReader::InputType::kPistol, {}),
        superstructure_goal_sender_(
            event_loop->MakeSender<superstructure::Goal>("/superstructure")),
        localizer_control_sender_(
            event_loop->MakeSender<
                ::frc971::control_loops::drivetrain::LocalizerControl>(
                "/drivetrain")),
        superstructure_status_fetcher_(
            event_loop->MakeFetcher<superstructure::Status>("/superstructure")),
        setpoint_fetcher_(
            event_loop->MakeFetcher<Setpoint>("/superstructure")) {}

  // Localizer reset positions are with front of robot pressed up against driver
  // station in the middle of the field side-to-side.
  void BlueResetLocalizer() {
    auto builder = localizer_control_sender_.MakeBuilder();

    frc971::control_loops::drivetrain::LocalizerControl::Builder
        localizer_control_builder = builder.MakeBuilder<
            frc971::control_loops::drivetrain::LocalizerControl>();
    localizer_control_builder.add_x(-7.9);
    localizer_control_builder.add_y(0.0);
    localizer_control_builder.add_theta_uncertainty(10.0);
    localizer_control_builder.add_theta(M_PI);
    localizer_control_builder.add_keep_current_theta(false);
    if (builder.Send(localizer_control_builder.Finish()) !=
        aos::RawSender::Error::kOk) {
      AOS_LOG(ERROR, "Failed to reset blue localizer.\n");
    }
  }

  void RedResetLocalizer() {
    auto builder = localizer_control_sender_.MakeBuilder();

    frc971::control_loops::drivetrain::LocalizerControl::Builder
        localizer_control_builder = builder.MakeBuilder<
            frc971::control_loops::drivetrain::LocalizerControl>();
    localizer_control_builder.add_x(7.9);
    localizer_control_builder.add_y(0.0);
    localizer_control_builder.add_theta_uncertainty(10.0);
    localizer_control_builder.add_theta(0.0);
    localizer_control_builder.add_keep_current_theta(false);
    if (builder.Send(localizer_control_builder.Finish()) !=
        aos::RawSender::Error::kOk) {
      AOS_LOG(ERROR, "Failed to reset red localizer.\n");
    }
  }

  void ResetLocalizer() {
    const frc971::control_loops::drivetrain::Status *drivetrain_status =
        this->drivetrain_status();
    if (drivetrain_status == nullptr) {
      return;
    }
    // Get the current position
    // Snap to heading.
    auto builder = localizer_control_sender_.MakeBuilder();

    // TODO<Henry> Put our starting location here.
    frc971::control_loops::drivetrain::LocalizerControl::Builder
        localizer_control_builder = builder.MakeBuilder<
            frc971::control_loops::drivetrain::LocalizerControl>();
    localizer_control_builder.add_x(drivetrain_status->x());
    localizer_control_builder.add_y(drivetrain_status->y());
    const double new_theta =
        frc971::zeroing::Wrap(drivetrain_status->theta(), 0, M_PI);
    localizer_control_builder.add_theta(new_theta);
    localizer_control_builder.add_theta_uncertainty(10.0);
    if (builder.Send(localizer_control_builder.Finish()) !=
        aos::RawSender::Error::kOk) {
      AOS_LOG(ERROR, "Failed to reset localizer.\n");
    }
  }

  void AutoEnded() override { AOS_LOG(INFO, "Auto ended.\n"); }

  void HandleTeleop(
      const ::frc971::input::driver_station::Data &data) override {
    superstructure_status_fetcher_.Fetch();
    if (!superstructure_status_fetcher_.get()) {
      AOS_LOG(ERROR, "Got no superstructure status message.\n");
      return;
    }

    setpoint_fetcher_.Fetch();

    // Default to the intakes in
    constexpr double kIntakeUpPosition = 1.47;
    double intake_front_pos = kIntakeUpPosition;
    double intake_back_pos = kIntakeUpPosition;
    double transfer_roller_speed = 0.0;
    std::optional<control_loops::superstructure::RequestedIntake>
        requested_intake;

    double roller_front_speed = 0.0;
    double roller_back_speed = 0.0;

    std::optional<double> turret_pos = std::nullopt;

    double climber_position = 0.01;
    bool climber_servo = false;

    double catapult_pos = 0.03;
    double catapult_speed = 18.0;
    double catapult_return_pos = 0.0;
    bool fire = false;

    if (data.PosEdge(kLocalizerReset)) {
      ResetLocalizer();
    }

    if (data.PosEdge(kRedLocalizerReset)) {
      RedResetLocalizer();
    }
    if (data.PosEdge(kBlueLocalizerReset)) {
      BlueResetLocalizer();
    }

    if (data.IsPressed(kClimberExtend)) {
      climber_position = 0.54;
    } else {
      climber_position = 0.005;
    }

    if (data.IsPressed(kClimberServo)) {
      climber_servo = true;
    }

    if (data.IsPressed(kTurret)) {
      if (setpoint_fetcher_.get()) {
        turret_pos = setpoint_fetcher_->turret();
      } else {
        turret_pos = -1.5;
      }
    }

    if (setpoint_fetcher_.get()) {
      catapult_pos = setpoint_fetcher_->catapult_position();
      catapult_speed = setpoint_fetcher_->catapult_velocity();
    }

    // Keep the catapult return position at the shot one if kCatapultPos is
    // pressed
    if (data.IsPressed(kCatapultPos)) {
      catapult_return_pos = 0.7;
    } else {
      catapult_return_pos = -0.908;
    }

    constexpr double kRollerSpeed = 12.0;
    constexpr double kTransferRollerSpeed = 12.0;
    constexpr double kIntakePosition = -0.12;
    constexpr size_t kIntakeCounterIterations = 25;

    if (data.PosEdge(kSpit)) {
      last_front_intake_has_ball_ =
          superstructure_status_fetcher_->front_intake_has_ball();
      last_back_intake_has_ball_ =
          superstructure_status_fetcher_->back_intake_has_ball();
    }

    // Extend the intakes and spin the rollers.
    // Don't let this happen if there is a ball in the other intake, because
    // that would spit this one out.
    if (data.IsPressed(kIntakeFrontOut) &&
        !superstructure_status_fetcher_->back_intake_has_ball()) {
      intake_front_pos = kIntakePosition;
      transfer_roller_speed = kTransferRollerSpeed;

      intake_front_counter_ = kIntakeCounterIterations;
      intake_back_counter_ = 0;
    } else if (data.IsPressed(kIntakeBackOut) &&
               !superstructure_status_fetcher_->front_intake_has_ball()) {
      intake_back_pos = kIntakePosition;
      transfer_roller_speed = -kTransferRollerSpeed;

      intake_back_counter_ = kIntakeCounterIterations;
      intake_front_counter_ = 0;
    } else if (data.IsPressed(kSpitFront) ||
               (data.IsPressed(kSpit) && last_front_intake_has_ball_)) {
      transfer_roller_speed = -kTransferRollerSpeed;
      intake_front_counter_ = 0;
    } else if (data.IsPressed(kSpitBack) ||
               (data.IsPressed(kSpit) && last_back_intake_has_ball_)) {
      transfer_roller_speed = kTransferRollerSpeed;
      intake_back_counter_ = 0;
    }

    // Keep spinning the rollers a bit after they let go
    if (intake_front_counter_ > 0) {
      intake_front_counter_--;
      roller_front_speed = kRollerSpeed;
      requested_intake = control_loops::superstructure::RequestedIntake::kFront;
    }
    if (intake_back_counter_ > 0) {
      intake_back_counter_--;
      roller_back_speed = kRollerSpeed;
      requested_intake = control_loops::superstructure::RequestedIntake::kBack;
    }

    if (data.IsPressed(kFire)) {
      fire = true;
      // Provide a default turret goal.
    }

    if (data.IsPressed(kClimberIntakes)) {
      intake_back_pos = kIntakeUpPosition;
      intake_front_pos = kIntakePosition;
    }

    {
      auto builder = superstructure_goal_sender_.MakeBuilder();

      flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
          intake_front_offset =
              CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
                  *builder.fbb(), intake_front_pos,
                  CreateProfileParameters(*builder.fbb(), 8.0, 40.0));
      flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
          intake_back_offset =
              CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
                  *builder.fbb(), intake_back_pos,
                  CreateProfileParameters(*builder.fbb(), 8.0, 40.0));

      flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
          turret_offset;
      if (turret_pos.has_value()) {
        turret_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
            *builder.fbb(), turret_pos.value(),
            CreateProfileParameters(*builder.fbb(), 12.0, 20.0));
      }

      flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
          catapult_return_offset =
              CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
                  *builder.fbb(), catapult_return_pos,
                  frc971::CreateProfileParameters(*builder.fbb(), 9.0, 50.0));

      flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
          climber_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
              *builder.fbb(), climber_position,
              frc971::CreateProfileParameters(*builder.fbb(), 1.0, 1.0));

      superstructure::CatapultGoal::Builder catapult_builder =
          builder.MakeBuilder<superstructure::CatapultGoal>();
      catapult_builder.add_return_position(catapult_return_offset);
      catapult_builder.add_shot_position(catapult_pos);
      catapult_builder.add_shot_velocity(catapult_speed);
      flatbuffers::Offset<superstructure::CatapultGoal> catapult_offset =
          catapult_builder.Finish();

      superstructure::Goal::Builder superstructure_goal_builder =
          builder.MakeBuilder<superstructure::Goal>();

      superstructure_goal_builder.add_intake_front(intake_front_offset);
      superstructure_goal_builder.add_intake_back(intake_back_offset);
      if (!turret_offset.IsNull()) {
        superstructure_goal_builder.add_turret(turret_offset);
      }
      superstructure_goal_builder.add_climber(climber_offset);
      superstructure_goal_builder.add_catapult(catapult_offset);
      superstructure_goal_builder.add_fire(fire);

      superstructure_goal_builder.add_roller_speed_front(roller_front_speed);
      superstructure_goal_builder.add_roller_speed_back(roller_back_speed);
      superstructure_goal_builder.add_transfer_roller_speed(
          transfer_roller_speed);
      superstructure_goal_builder.add_climber_servo(climber_servo);
      superstructure_goal_builder.add_auto_aim(data.IsPressed(kAutoAim));
      if (requested_intake.has_value()) {
        superstructure_goal_builder.add_turret_intake(requested_intake.value());
      }

      if (builder.Send(superstructure_goal_builder.Finish()) !=
          aos::RawSender::Error::kOk) {
        AOS_LOG(ERROR, "Sending superstructure goal failed.\n");
      }
    }
  }

 private:
  ::aos::Sender<superstructure::Goal> superstructure_goal_sender_;

  ::aos::Sender<frc971::control_loops::drivetrain::LocalizerControl>
      localizer_control_sender_;

  ::aos::Fetcher<superstructure::Status> superstructure_status_fetcher_;

  ::aos::Fetcher<Setpoint> setpoint_fetcher_;

  size_t intake_front_counter_ = 0;
  size_t intake_back_counter_ = 0;

  bool last_front_intake_has_ball_ = false;
  bool last_back_intake_has_ball_ = false;
};

}  // namespace joysticks
}  // namespace input
}  // namespace y2022

int main(int argc, char **argv) {
  ::aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("aos_config.json");

  ::aos::ShmEventLoop event_loop(&config.message());
  ::y2022::input::joysticks::Reader reader(&event_loop);

  event_loop.Run();

  return 0;
}
