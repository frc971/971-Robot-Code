#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "aos/actions/actions.h"
#include "aos/init.h"
#include "aos/input/action_joystick_input.h"
#include "aos/input/driver_station_data.h"
#include "aos/input/drivetrain_input.h"
#include "aos/logging/logging.h"
#include "aos/time/time.h"
#include "aos/util/log_interval.h"
#include "frc971/autonomous/base_autonomous_actor.h"
#include "y2017/constants.h"
#include "y2017/control_loops/drivetrain/drivetrain_base.h"
#include "y2017/control_loops/superstructure/superstructure_goal_generated.h"
#include "y2017/control_loops/superstructure/superstructure_status_generated.h"

using ::aos::input::driver_station::ButtonLocation;
using ::aos::input::driver_station::ControlBit;
using ::aos::input::driver_station::JoystickAxis;
using ::aos::input::driver_station::POVLocation;
using ::aos::input::DrivetrainInputReader;

namespace y2017 {
namespace input {
namespace joysticks {

namespace superstructure = control_loops::superstructure;

const ButtonLocation kGearSlotBack(2, 11);

const ButtonLocation kIntakeDown(3, 9);
const POVLocation kIntakeUp(3, 90);
const ButtonLocation kIntakeIn(3, 12);
const ButtonLocation kIntakeOut(3, 8);
const ButtonLocation kFire(3, 3);
const ButtonLocation kVisionDistanceShot(3, 7);
const ButtonLocation kMiddleShot(3, 6);
const POVLocation kFarShot(3, 270);

const ButtonLocation kVisionAlign(3, 5);

const ButtonLocation kReverseIndexer(3, 4);
const ButtonLocation kExtra1(3, 11);
const ButtonLocation kExtra2(3, 10);
const ButtonLocation kHang(3, 2);

class Reader : public ::aos::input::ActionJoystickInput {
 public:
  Reader(::aos::EventLoop *event_loop)
      : ::aos::input::ActionJoystickInput(
            event_loop,
            ::y2017::control_loops::drivetrain::GetDrivetrainConfig(),
            DrivetrainInputReader::InputType::kSteeringWheel, {}),
        superstructure_status_fetcher_(
            event_loop
                ->MakeFetcher<::y2017::control_loops::superstructure::Status>(
                    "/superstructure")),
        superstructure_goal_sender_(
            event_loop
                ->MakeSender<::y2017::control_loops::superstructure::Goal>(
                    "/superstructure")) {}

  enum class ShotDistance { VISION_SHOT, MIDDLE_SHOT, FAR_SHOT };

  void HandleTeleop(const ::aos::input::driver_station::Data &data) {
    // Default the intake to in.
    intake_goal_ = 0.07;
    bool lights_on = false;
    bool vision_track = false;

    superstructure_status_fetcher_.Fetch();
    if (!superstructure_status_fetcher_.get()) {
      AOS_LOG(ERROR, "Got no superstructure status packet.\n");
      return;
    }

    if (data.IsPressed(kIntakeUp)) {
      intake_goal_ = 0.0;
      turret_goal_ = M_PI / 3.0;
    }
    if (data.IsPressed(kIntakeDown)) {
      intake_goal_ = 0.235;
      // Don't go quite so far out since we have a gear mech out now.
      if (data.IsPressed(kIntakeUp)) {
        intake_goal_ = 0.160;
      }
    }

    if (data.IsPressed(kVisionAlign)) {
      // Align shot using vision
      // TODO(campbell): Add vision aligning.
      lights_on = true;
      vision_track = true;
    }
    if (data.PosEdge(kMiddleShot)) {
      turret_goal_ = -M_PI;
    }
    if (data.PosEdge(kFarShot)) {
      turret_goal_ = 0.0;
    }
    if (data.PosEdge(kVisionDistanceShot)) {
      turret_goal_ = 0.0;
    }

    if (data.IsPressed(kVisionDistanceShot)) {
      last_shot_distance_ = ShotDistance::VISION_SHOT;
    } else if (data.IsPressed(kMiddleShot)) {
      last_shot_distance_ = ShotDistance::MIDDLE_SHOT;
    } else if (data.IsPressed(kFarShot)) {
      last_shot_distance_ = ShotDistance::FAR_SHOT;
    }

    if (data.IsPressed(kVisionAlign) ||
        data.IsPressed(kMiddleShot) || data.IsPressed(kFarShot) ||
        data.IsPressed(kFire)) {
      switch (last_shot_distance_) {
        case ShotDistance::VISION_SHOT:
          hood_goal_ = 0.10;
          shooter_velocity_ = 300.0;

          vision_distance_shot_ = true;
          break;
        case ShotDistance::MIDDLE_SHOT:
          hood_goal_ = 0.43 - 0.00;
          shooter_velocity_ = 364.0;
          vision_distance_shot_ = false;
          break;
        case ShotDistance::FAR_SHOT:
          hood_goal_ = 0.47;
          shooter_velocity_ = 410.0;
          vision_distance_shot_ = false;
          break;
      }
    } else {
      //hood_goal_ = 0.15;
      shooter_velocity_ = 0.0;
    }

    if (data.IsPressed(kExtra1)) {
      //turret_goal_ = -M_PI * 3.0 / 4.0;
      turret_goal_ += 0.150;
    }
    if (data.IsPressed(kExtra2)) {
      //turret_goal_ = M_PI * 3.0 / 4.0;
      turret_goal_ -= 0.150;
    }
    turret_goal_ = ::std::max(::std::min(turret_goal_, M_PI), -M_PI);

    fire_ = data.IsPressed(kFire) && shooter_velocity_ != 0.0;
    if (data.IsPressed(kVisionAlign)) {
      fire_ = fire_ && superstructure_status_fetcher_->turret()->vision_tracking();
    }

    auto builder = superstructure_goal_sender_.MakeBuilder();
    if (data.IsPressed(kHang)) {
      intake_goal_ = 0.23;
    }

    bool intake_disable_intake = false;
    double intake_gear_servo = 0.0;

    if (data.IsPressed(kIntakeUp)) {
      intake_gear_servo = 0.30;
    } else {
      // Clamp the gear
      intake_gear_servo = 0.66;
    }

    double intake_voltage_rollers = 0.0;

    if (superstructure_status_fetcher_->intake()->position() >
        superstructure_status_fetcher_->intake()->unprofiled_goal_position() +
            0.01) {
      intake_accumulator_ = 10;
    }
    if (intake_accumulator_ > 0) {
      --intake_accumulator_;
      if (!superstructure_status_fetcher_->intake()->estopped()) {
        intake_voltage_rollers = 10.0;
      }
    }

    if (data.IsPressed(kHang)) {
      intake_voltage_rollers = -10.0;
      intake_disable_intake = true;
    } else if (data.IsPressed(kIntakeIn) || data.IsPressed(kFire)) {
      if (robot_velocity() > 2.0) {
        intake_voltage_rollers = 12.0;
      } else {
        intake_voltage_rollers = 11.0;
      }
    } else if (data.IsPressed(kIntakeOut)) {
      intake_voltage_rollers = -8.0;
    }
    if (intake_goal_ < 0.1) {
      intake_voltage_rollers = ::std::min(8.0, intake_voltage_rollers);
    }

    double indexer_voltage_rollers = 0.0;
    double indexer_angular_velocity = 0.0;
    if (data.IsPressed(kReverseIndexer)) {
      indexer_voltage_rollers = -12.0;
      indexer_angular_velocity = 1.0;
    } else if (fire_) {
      indexer_voltage_rollers = 12.0;
      switch (last_shot_distance_)  {
        case ShotDistance::VISION_SHOT:
          indexer_angular_velocity = -1.25 * M_PI;
          break;
        case ShotDistance::MIDDLE_SHOT:
        case ShotDistance::FAR_SHOT:
          indexer_angular_velocity = -2.25 * M_PI;
          break;
      }
    } else {
      indexer_voltage_rollers = 0.0;
      indexer_angular_velocity = 0.0;
    }

    superstructure::IndexerGoal::Builder indexer_goal_builder =
        builder.MakeBuilder<superstructure::IndexerGoal>();
    indexer_goal_builder.add_angular_velocity(indexer_angular_velocity);
    indexer_goal_builder.add_voltage_rollers(indexer_voltage_rollers);

    flatbuffers::Offset<superstructure::IndexerGoal> indexer_goal_offset =
        indexer_goal_builder.Finish();

    flatbuffers::Offset<frc971::ProfileParameters>
        turret_profile_parameters_offset;
    if (vision_track) {
      turret_profile_parameters_offset =
          frc971::CreateProfileParameters(*builder.fbb(), 10.0, 35.0);
    } else {
      turret_profile_parameters_offset =
          frc971::CreateProfileParameters(*builder.fbb(), 6.0, 15.0);
    }
    superstructure::TurretGoal::Builder turret_goal_builder =
        builder.MakeBuilder<superstructure::TurretGoal>();
    turret_goal_builder.add_angle(turret_goal_);
    turret_goal_builder.add_track(vision_track);
    turret_goal_builder.add_profile_params(turret_profile_parameters_offset);
    flatbuffers::Offset<superstructure::TurretGoal> turret_goal_offset =
        turret_goal_builder.Finish();

    flatbuffers::Offset<frc971::ProfileParameters>
        intake_profile_parameters_offset =
          frc971::CreateProfileParameters(*builder.fbb(), 0.5, 3.0);

    superstructure::IntakeGoal::Builder intake_goal_builder =
        builder.MakeBuilder<superstructure::IntakeGoal>();
    intake_goal_builder.add_voltage_rollers(intake_voltage_rollers);
    intake_goal_builder.add_distance(intake_goal_);
    intake_goal_builder.add_disable_intake(intake_disable_intake);
    intake_goal_builder.add_gear_servo(intake_gear_servo);
    intake_goal_builder.add_profile_params(intake_profile_parameters_offset);
    flatbuffers::Offset<superstructure::IntakeGoal> intake_goal_offset =
        intake_goal_builder.Finish();

    flatbuffers::Offset<frc971::ProfileParameters>
        hood_profile_parameters_offset =
            frc971::CreateProfileParameters(*builder.fbb(), 5.0, 25.0);

    superstructure::HoodGoal::Builder hood_goal_builder =
        builder.MakeBuilder<superstructure::HoodGoal>();
    hood_goal_builder.add_angle(hood_goal_);
    hood_goal_builder.add_profile_params(hood_profile_parameters_offset);
    flatbuffers::Offset<superstructure::HoodGoal> hood_goal_offset =
        hood_goal_builder.Finish();

    superstructure::ShooterGoal::Builder shooter_goal_builder =
        builder.MakeBuilder<superstructure::ShooterGoal>();
    shooter_goal_builder.add_angular_velocity(shooter_velocity_);
    flatbuffers::Offset<superstructure::ShooterGoal> shooter_goal_offset =
        shooter_goal_builder.Finish();

    superstructure::Goal::Builder goal_builder =
        builder.MakeBuilder<superstructure::Goal>();
    goal_builder.add_lights_on(lights_on);
    if (data.IsPressed(kVisionAlign) && vision_distance_shot_) {
      goal_builder.add_use_vision_for_shots(true);
    } else {
      goal_builder.add_use_vision_for_shots(false);
    }

    goal_builder.add_intake(intake_goal_offset);
    goal_builder.add_indexer(indexer_goal_offset);
    goal_builder.add_turret(turret_goal_offset);
    goal_builder.add_hood(hood_goal_offset);
    goal_builder.add_shooter(shooter_goal_offset);

    if (!builder.Send(goal_builder.Finish())) {
      AOS_LOG(ERROR, "Sending superstructure goal failed.\n");
    }
  }

 private:
  ::aos::Fetcher<::y2017::control_loops::superstructure::Status>
      superstructure_status_fetcher_;
  ::aos::Sender<::y2017::control_loops::superstructure::Goal>
      superstructure_goal_sender_;

  ShotDistance last_shot_distance_ = ShotDistance::VISION_SHOT;

  // Current goals to send to the robot.
  double intake_goal_ = 0.0;
  double turret_goal_ = 0.0;
  double hood_goal_ = 0.3;
  double shooter_velocity_ = 0.0;
  int intake_accumulator_ = 0;

  bool vision_distance_shot_ = false;

  bool fire_ = false;
};

}  // namespace joysticks
}  // namespace input
}  // namespace y2017

int main() {
  ::aos::InitNRT(true);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("config.json");

  ::aos::ShmEventLoop event_loop(&config.message());
  ::y2017::input::joysticks::Reader reader(&event_loop);

  event_loop.Run();

  ::aos::Cleanup();
}
