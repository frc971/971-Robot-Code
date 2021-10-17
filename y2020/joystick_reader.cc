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
#include "frc971/input/action_joystick_input.h"
#include "frc971/input/driver_station_data.h"
#include "frc971/input/drivetrain_input.h"
#include "frc971/input/joystick_input.h"
#include "y2020/constants.h"
#include "y2020/control_loops/drivetrain/drivetrain_base.h"
#include "y2020/control_loops/superstructure/superstructure_goal_generated.h"
#include "y2020/control_loops/superstructure/superstructure_status_generated.h"
#include "y2020/setpoint_generated.h"

using frc971::input::driver_station::ButtonLocation;
using frc971::input::driver_station::ControlBit;
using frc971::input::driver_station::JoystickAxis;
using frc971::input::driver_station::POVLocation;

using frc971::CreateProfileParameters;
using frc971::control_loops::CreateStaticZeroingSingleDOFProfiledSubsystemGoal;
using frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoal;

namespace y2020 {
namespace input {
namespace joysticks {

namespace superstructure = y2020::control_loops::superstructure;

// TODO(sabina): fix button locations.

const ButtonLocation kShootFast(3, 16);
const ButtonLocation kAutoTrack(3, 3);
const ButtonLocation kAutoNoHood(3, 5);
const ButtonLocation kHood(3, 4);
const ButtonLocation kShootSlow(4, 2);
const ButtonLocation kFeed(4, 1);
const ButtonLocation kFeedDriver(1, 2);
const ButtonLocation kIntakeExtend(3, 9);
const ButtonLocation kIntakeIn(4, 4);
const ButtonLocation kSpit(4, 3);
const ButtonLocation kLocalizerReset(3, 8);

const ButtonLocation kWinch(3, 14);

class Reader : public ::frc971::input::ActionJoystickInput {
 public:
  Reader(::aos::EventLoop *event_loop)
      : ::frc971::input::ActionJoystickInput(
            event_loop,
            ::y2020::control_loops::drivetrain::GetDrivetrainConfig(),
            ::frc971::input::DrivetrainInputReader::InputType::kPistol, {}),
        superstructure_goal_sender_(
            event_loop->MakeSender<superstructure::Goal>("/superstructure")),
        localizer_control_sender_(
            event_loop->MakeSender<
                ::frc971::control_loops::drivetrain::LocalizerControl>(
                "/drivetrain")),
        superstructure_status_fetcher_(
            event_loop->MakeFetcher<superstructure::Status>("/superstructure")),
        setpoint_fetcher_(event_loop->MakeFetcher<y2020::joysticks::Setpoint>(
            "/superstructure")) {}

  void AutoEnded() override {
    AOS_LOG(INFO, "Auto ended, assuming disc and have piece\n");
  }

  void ResetLocalizer() {
    auto builder = localizer_control_sender_.MakeBuilder();

    // Start roughly in front of the red-team goal, robot pointed away from
    // goal.
    frc971::control_loops::drivetrain::LocalizerControl::Builder
        localizer_control_builder = builder.MakeBuilder<
            frc971::control_loops::drivetrain::LocalizerControl>();
    localizer_control_builder.add_x(5.0);
    localizer_control_builder.add_y(-2.0);
    localizer_control_builder.add_theta(M_PI);
    localizer_control_builder.add_theta_uncertainty(0.01);
    if (!builder.Send(localizer_control_builder.Finish())) {
      AOS_LOG(ERROR, "Failed to reset localizer.\n");
    }
  }

  void HandleTeleop(
      const ::frc971::input::driver_station::Data &data) override {
    superstructure_status_fetcher_.Fetch();
    if (!superstructure_status_fetcher_.get()) {
      AOS_LOG(ERROR, "Got no superstructure status message.\n");
      return;
    }

    setpoint_fetcher_.Fetch();

    double hood_pos = constants::Values::kHoodRange().middle();
    double intake_pos = -0.89;
    double turret_pos = 0.0;
    float roller_speed = 0.0f;
    float roller_speed_compensation = 0.0f;
    double accelerator_speed = 0.0;
    double finisher_speed = 0.0;
    double climber_speed = 0.0;
    bool preload_intake = false;

    const bool auto_track =
        data.IsPressed(kAutoTrack) || data.IsPressed(kAutoNoHood);

    if (data.IsPressed(kHood)) {
      hood_pos = 0.45;
    } else {
      if (setpoint_fetcher_.get()) {
        hood_pos = setpoint_fetcher_->hood();
      } else {
        hood_pos = 0.58;
      }
    }

    if (setpoint_fetcher_.get()) {
      turret_pos = setpoint_fetcher_->turret();
    } else {
      turret_pos = 0.0;
    }

    if (data.IsPressed(kShootFast)) {
      if (setpoint_fetcher_.get()) {
        accelerator_speed = setpoint_fetcher_->accelerator();
        finisher_speed = setpoint_fetcher_->finisher();
      } else {
        accelerator_speed = 250.0;
        finisher_speed = 500.0;
      }
    } else if (data.IsPressed(kShootSlow)) {
      accelerator_speed = 180.0;
      finisher_speed = 300.0;
    }

    if (data.IsPressed(kIntakeExtend)) {
      intake_pos = 1.2;
      roller_speed = 7.0f;
      roller_speed_compensation = 2.0f;
      preload_intake = true;
    }

    if (superstructure_status_fetcher_.get() &&
        superstructure_status_fetcher_->intake()->zeroed() &&
        superstructure_status_fetcher_->intake()->position() > -0.5) {
      roller_speed = std::max(roller_speed, 6.0f);
      roller_speed_compensation = 2.0f;
    }

    if (data.IsPressed(kIntakeIn)) {
      roller_speed = 6.0f;
      roller_speed_compensation = 2.0f;
      preload_intake = true;
    } else if (data.IsPressed(kSpit)) {
      roller_speed = -6.0f;
    }

    if (data.IsPressed(kWinch)) {
      climber_speed = 12.0f;
    }

    if (data.IsPressed(kLocalizerReset)) {
      ResetLocalizer();
    }

    auto builder = superstructure_goal_sender_.MakeBuilder();

    flatbuffers::Offset<superstructure::Goal> superstructure_goal_offset;
    {
      flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
          hood_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
              *builder.fbb(), hood_pos,
              CreateProfileParameters(*builder.fbb(), 5.0, 30.0));

      flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
          intake_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
              *builder.fbb(), intake_pos,
              CreateProfileParameters(*builder.fbb(), 10.0, 30.0));

      flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
          turret_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
              *builder.fbb(), turret_pos,
              CreateProfileParameters(*builder.fbb(), 6.0, 20.0));

      flatbuffers::Offset<superstructure::ShooterGoal> shooter_offset =
          superstructure::CreateShooterGoal(*builder.fbb(), accelerator_speed,
                                            finisher_speed);

      superstructure::Goal::Builder superstructure_goal_builder =
          builder.MakeBuilder<superstructure::Goal>();

      superstructure_goal_builder.add_hood(hood_offset);
      superstructure_goal_builder.add_intake(intake_offset);
      superstructure_goal_builder.add_turret(turret_offset);
      superstructure_goal_builder.add_roller_voltage(roller_speed);
      superstructure_goal_builder.add_roller_speed_compensation(
          roller_speed_compensation);
      superstructure_goal_builder.add_shooter(shooter_offset);
      superstructure_goal_builder.add_shooting(data.IsPressed(kFeed) ||
                                               data.IsPressed(kFeedDriver));
      superstructure_goal_builder.add_climber_voltage(climber_speed);

      superstructure_goal_builder.add_turret_tracking(auto_track);
      superstructure_goal_builder.add_hood_tracking(
          auto_track && !data.IsPressed(kAutoNoHood));
      superstructure_goal_builder.add_shooter_tracking(auto_track);
      superstructure_goal_builder.add_intake_preloading(preload_intake);

      if (!builder.Send(superstructure_goal_builder.Finish())) {
        AOS_LOG(ERROR, "Sending superstructure goal failed.\n");
      }
    }
  }

 private:
  ::aos::Sender<superstructure::Goal> superstructure_goal_sender_;

  ::aos::Sender<frc971::control_loops::drivetrain::LocalizerControl>
      localizer_control_sender_;

  ::aos::Fetcher<superstructure::Status> superstructure_status_fetcher_;

  ::aos::Fetcher<y2020::joysticks::Setpoint> setpoint_fetcher_;
};

}  // namespace joysticks
}  // namespace input
}  // namespace y2020

int main(int argc, char **argv) {
  ::aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("config.json");

  ::aos::ShmEventLoop event_loop(&config.message());
  ::y2020::input::joysticks::Reader reader(&event_loop);

  event_loop.Run();

  return 0;
}
