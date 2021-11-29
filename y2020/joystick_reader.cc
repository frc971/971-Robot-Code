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
#include "frc971/zeroing/wrap.h"
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

const ButtonLocation kAutoTrack(3, 3);
const ButtonLocation kAutoNoHood(3, 5);
const ButtonLocation kHood(3, 2);
const ButtonLocation kShootSlow(4, 2);
const ButtonLocation kFixedTurret(3, 1);
const ButtonLocation kFeed(4, 1);
const ButtonLocation kFeedDriver(1, 2);
const ButtonLocation kIntakeExtend(3, 9);
const ButtonLocation kIntakeExtendDriver(1, 4);
const ButtonLocation kRedLocalizerReset(3, 13);
const ButtonLocation kBlueLocalizerReset(3, 14);
const ButtonLocation kIntakeIn(4, 4);
const ButtonLocation kSpit(4, 3);
const ButtonLocation kLocalizerReset(3, 8);
const ButtonLocation kIntakeSlightlyOut(3, 7);

const ButtonLocation kWinch(3, 4);
const ButtonLocation kUnWinch(3, 6);

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

    // Start roughly in front of the red-team goal, robot pointed away from
    // goal.
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
    bool roller_speed_compensation = false;
    double accelerator_speed = 0.0;
    double finisher_speed = 0.0;
    double climber_speed = 0.0;
    bool preload_intake = false;
    bool turret_tracking = false;

    const bool auto_track = data.IsPressed(kAutoTrack);

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

    if (!data.IsPressed(kFixedTurret)) {
      turret_tracking = true;
    }

    if (data.IsPressed(kAutoNoHood)) {
      if (setpoint_fetcher_.get()) {
        accelerator_speed = setpoint_fetcher_->accelerator();
        finisher_speed = setpoint_fetcher_->finisher();
      }
    } else if (data.IsPressed(kShootSlow)) {
      accelerator_speed = 400.0;
      finisher_speed = 200.0;
    }

    if (data.IsPressed(kIntakeExtend) || data.IsPressed(kIntakeExtendDriver)) {
      intake_pos = 1.24;
      roller_speed = 7.0f;
      roller_speed_compensation = true;
      preload_intake = true;
    }

    if (superstructure_status_fetcher_.get() &&
        superstructure_status_fetcher_->intake()->zeroed() &&
        superstructure_status_fetcher_->intake()->position() > -0.5) {
      roller_speed = std::max(roller_speed, 6.0f);
      roller_speed_compensation = true;
    }

    if (data.IsPressed(kIntakeIn)) {
      roller_speed = 5.0f;
      roller_speed_compensation = true;
      preload_intake = true;
    } else if (data.IsPressed(kSpit)) {
      roller_speed = -6.0f;
    } else if (data.IsPressed(kIntakeSlightlyOut)) {
      intake_pos = -0.426585;
      roller_speed = 6.0f;
      preload_intake = true;
    }

    if (data.IsPressed(kWinch)) {
      ++winch_counter_;
    } else {
      winch_counter_ = 0;
    }

    if (winch_counter_ > 5 || (winch_counter_ > 0 && latched_climbing_)) {
      climber_speed = 12.0f;
      latched_climbing_ = true;
    }

    if (data.IsPressed(kUnWinch)) {
      ++unwinch_counter_;
    } else {
      unwinch_counter_ = 0;
    }

    if (unwinch_counter_ > 10 || (unwinch_counter_ > 0 && latched_climbing_)) {
      climber_speed = -12.0f;
      latched_climbing_ = true;
    }

    if (data.IsPressed(kWinch) && data.IsPressed(kUnWinch)) {
      latched_climbing_ = false;
      unwinch_counter_ = 0;
      winch_counter_ = 0;
    }

    if (latched_climbing_) {
      turret_tracking = false;
      turret_pos = -M_PI / 2.0;
    }

    if (data.PosEdge(kLocalizerReset)) {
      ResetLocalizer();
    }

    if (data.PosEdge(kRedLocalizerReset)) {
      RedResetLocalizer();
    }
    if (data.PosEdge(kBlueLocalizerReset)) {
      BlueResetLocalizer();
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
              CreateProfileParameters(*builder.fbb(), 20.0, 70.0));

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
          roller_speed_compensation ? 1.5f : 0.0f);
      superstructure_goal_builder.add_shooter(shooter_offset);
      superstructure_goal_builder.add_shooting(data.IsPressed(kFeed) ||
                                               data.IsPressed(kFeedDriver));
      if (data.IsPressed(kSpit)) {
        superstructure_goal_builder.add_feed_voltage_override(-7.0);
      }
      superstructure_goal_builder.add_climber_voltage(climber_speed);

      superstructure_goal_builder.add_turret_tracking(turret_tracking);
      superstructure_goal_builder.add_hood_tracking(
          !data.IsPressed(kFixedTurret) && !data.IsPressed(kAutoNoHood));
      superstructure_goal_builder.add_shooter_tracking(
          auto_track ||
          (!data.IsPressed(kFixedTurret) && !data.IsPressed(kAutoNoHood) &&
           data.IsPressed(kFeedDriver)));
      superstructure_goal_builder.add_intake_preloading(preload_intake);

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

  ::aos::Fetcher<y2020::joysticks::Setpoint> setpoint_fetcher_;

  bool latched_climbing_ = false;

  size_t winch_counter_ = 0;
  size_t unwinch_counter_ = 0;
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
