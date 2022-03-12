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

// TODO(henry) put actually button locations here
// TODO(milind): integrate with shooting statemachine and aimer
#if 0
const ButtonLocation kCatapultPos(4, 3);
const ButtonLocation kFire(3, 4);
const ButtonLocation kTurret(4, 15);

const ButtonLocation kIntakeFrontOut(4, 10);
const ButtonLocation kIntakeBackOut(4, 9);

const ButtonLocation kRedLocalizerReset(3, 13);
const ButtonLocation kBlueLocalizerReset(3, 14);
const ButtonLocation kLocalizerReset(3, 8);
#else

const ButtonLocation kCatapultPos(4, 3);
const ButtonLocation kFire(4, 1);
const ButtonLocation kTurret(4, 15);

const ButtonLocation kIntakeFrontOut(4, 10);
const ButtonLocation kIntakeBackOut(4, 9);

const ButtonLocation kRedLocalizerReset(3, 13);
const ButtonLocation kBlueLocalizerReset(3, 14);
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

  void BlueResetLocalizer() {
    auto builder = localizer_control_sender_.MakeBuilder();

    frc971::control_loops::drivetrain::LocalizerControl::Builder
        localizer_control_builder = builder.MakeBuilder<
            frc971::control_loops::drivetrain::LocalizerControl>();
    localizer_control_builder.add_x(7.4);
    localizer_control_builder.add_y(-1.7);
    localizer_control_builder.add_theta_uncertainty(10.0);
    localizer_control_builder.add_theta(0.0);
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
    localizer_control_builder.add_x(-7.4);
    localizer_control_builder.add_y(1.7);
    localizer_control_builder.add_theta_uncertainty(10.0);
    localizer_control_builder.add_theta(M_PI);
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
    double intake_front_pos = 1.47;
    double intake_back_pos = 1.47;
    double transfer_roller_front_speed = 0.0;
    double transfer_roller_back_speed = 0.0;

    double roller_front_speed = 0.0;
    double roller_back_speed = 0.0;

    double turret_pos = 0.0;

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

    if (data.IsPressed(kTurret)) {
      if (setpoint_fetcher_.get()) {
        turret_pos = setpoint_fetcher_->turret();
      } else {
        turret_pos = -1.5;
      }
    } else {
      turret_pos = 0.0;
    }

    if (setpoint_fetcher_.get()) {
      catapult_pos = setpoint_fetcher_->catapult_position();
      catapult_speed = setpoint_fetcher_->catapult_velocity();
    }

    // Keep the catapult return position at the shot one if kCatapultPos is
    // pressed
    if (data.IsPressed(kCatapultPos)) {
      catapult_return_pos = 0.3;
    } else {
      catapult_return_pos = -0.908;
    }

    constexpr double kRollerSpeed = 8.0;
    constexpr size_t kIntakeCounterIterations = 25;

    // Extend the intakes and spin the rollers
    if (data.IsPressed(kIntakeFrontOut)) {
      intake_front_pos = 0.0;
      transfer_roller_front_speed = 12.0;
      transfer_roller_back_speed = -transfer_roller_front_speed;

      intake_front_counter_ = kIntakeCounterIterations;
    } else if (data.IsPressed(kIntakeBackOut)) {
      intake_back_pos = 0.0;
      transfer_roller_back_speed = 12.0;
      transfer_roller_front_speed = -transfer_roller_back_speed;

      intake_back_counter_ = kIntakeCounterIterations;
    }

    // Keep spinning the rollers a bit after they let go
    if (intake_front_counter_ > 0) {
      intake_front_counter_--;
      roller_front_speed = kRollerSpeed;
    }
    if (intake_back_counter_ > 0) {
      intake_back_counter_--;
      roller_back_speed = kRollerSpeed;
    }

    if (data.IsPressed(kFire)) {
      fire = true;
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
          turret_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
              *builder.fbb(), turret_pos,
              CreateProfileParameters(*builder.fbb(), 1.0, 10.0));

      flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
          catapult_return_offset =
              CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
                  *builder.fbb(), catapult_return_pos,
                  frc971::CreateProfileParameters(*builder.fbb(), 9.0, 50.0));

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
      superstructure_goal_builder.add_turret(turret_offset);
      superstructure_goal_builder.add_catapult(catapult_offset);
      superstructure_goal_builder.add_fire(fire);

      superstructure_goal_builder.add_roller_speed_front(roller_front_speed);
      superstructure_goal_builder.add_roller_speed_back(roller_back_speed);
      superstructure_goal_builder.add_transfer_roller_speed_front(
          transfer_roller_front_speed);
      superstructure_goal_builder.add_transfer_roller_speed_back(
          transfer_roller_back_speed);

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
