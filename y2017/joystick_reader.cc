#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "aos/actions/actions.h"
#include "aos/input/driver_station_data.h"
#include "aos/logging/logging.h"
#include "aos/time/time.h"
#include "aos/util/log_interval.h"
#include "aos/input/drivetrain_input.h"
#include "aos/input/joystick_input.h"
#include "aos/init.h"
#include "frc971/autonomous/auto.q.h"
#include "frc971/autonomous/base_autonomous_actor.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "y2017/constants.h"
#include "y2017/control_loops/superstructure/superstructure.q.h"
#include "y2017/control_loops/drivetrain/drivetrain_base.h"

using ::frc971::control_loops::drivetrain_queue;
using ::y2017::control_loops::superstructure_queue;

using ::aos::input::driver_station::ButtonLocation;
using ::aos::input::driver_station::ControlBit;
using ::aos::input::driver_station::JoystickAxis;
using ::aos::input::driver_station::POVLocation;
using ::aos::input::DrivetrainInputReader;

namespace y2017 {
namespace input {
namespace joysticks {

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

std::unique_ptr<DrivetrainInputReader> drivetrain_input_reader_;

class Reader : public ::aos::input::JoystickInput {
 public:
  Reader(::aos::EventLoop *event_loop)
      : ::aos::input::JoystickInput(event_loop),
        autonomous_action_factory_(
            ::frc971::autonomous::BaseAutonomousActor::MakeFactory(
                event_loop)) {
    drivetrain_input_reader_ = DrivetrainInputReader::Make(
        DrivetrainInputReader::InputType::kSteeringWheel,
        ::y2017::control_loops::drivetrain::GetDrivetrainConfig());
  }

  enum class ShotDistance { VISION_SHOT, MIDDLE_SHOT, FAR_SHOT };

  ShotDistance last_shot_distance_ = ShotDistance::VISION_SHOT;

  void RunIteration(const ::aos::input::driver_station::Data &data) override {
    bool last_auto_running = auto_running_;
    auto_running_ = data.GetControlBit(ControlBit::kAutonomous) &&
                    data.GetControlBit(ControlBit::kEnabled);
    if (auto_running_ != last_auto_running) {
      if (auto_running_) {
        StartAuto();
      } else {
        StopAuto();
      }
    }

    if (!auto_running_) {
      HandleDrivetrain(data);
      HandleTeleop(data);
    }

    // Process any pending actions.
    action_queue_.Tick();
    was_running_ = action_queue_.Running();
  }
  int intake_accumulator_ = 0;

  void HandleDrivetrain(const ::aos::input::driver_station::Data &data) {
    drivetrain_input_reader_->HandleDrivetrain(data);
    robot_velocity_ = drivetrain_input_reader_->robot_velocity();
  }

  void HandleTeleop(const ::aos::input::driver_station::Data &data) {
    // Default the intake to in.
    intake_goal_ = 0.07;
    bool lights_on = false;
    bool vision_track = false;

    if (!data.GetControlBit(ControlBit::kEnabled)) {
      action_queue_.CancelAllActions();
      LOG(DEBUG, "Canceling\n");
    }

    superstructure_queue.status.FetchLatest();
    if (!superstructure_queue.status.get()) {
      LOG(ERROR, "Got no superstructure status packet.\n");
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
      fire_ = fire_ && superstructure_queue.status->turret.vision_tracking;
    }

    auto new_superstructure_goal = superstructure_queue.goal.MakeMessage();
    if (data.IsPressed(kHang)) {
      intake_goal_ = 0.23;
    }

    new_superstructure_goal->intake.distance = intake_goal_;
    new_superstructure_goal->intake.disable_intake = false;
    new_superstructure_goal->turret.angle = turret_goal_;
    new_superstructure_goal->turret.track = vision_track;
    new_superstructure_goal->hood.angle = hood_goal_;
    new_superstructure_goal->shooter.angular_velocity = shooter_velocity_;

    if (data.IsPressed(kIntakeUp)) {
      new_superstructure_goal->intake.gear_servo = 0.30;
    } else {
      // Clamp the gear
      new_superstructure_goal->intake.gear_servo = 0.66;
    }

    new_superstructure_goal->intake.profile_params.max_velocity = 0.50;
    new_superstructure_goal->hood.profile_params.max_velocity = 5.0;

    new_superstructure_goal->intake.profile_params.max_acceleration = 3.0;
    if (vision_track) {
      new_superstructure_goal->turret.profile_params.max_acceleration = 35.0;
      new_superstructure_goal->turret.profile_params.max_velocity = 10.0;
    } else {
      new_superstructure_goal->turret.profile_params.max_acceleration = 15.0;
      new_superstructure_goal->turret.profile_params.max_velocity = 6.0;
    }
    new_superstructure_goal->hood.profile_params.max_acceleration = 25.0;

    new_superstructure_goal->intake.voltage_rollers = 0.0;
    new_superstructure_goal->lights_on = lights_on;

    if (data.IsPressed(kVisionAlign) && vision_distance_shot_) {
      new_superstructure_goal->use_vision_for_shots = true;
    } else {
      new_superstructure_goal->use_vision_for_shots = false;
    }

    if (superstructure_queue.status->intake.position >
        superstructure_queue.status->intake.unprofiled_goal_position + 0.01) {
      intake_accumulator_ = 10;
    }
    if (intake_accumulator_ > 0) {
      --intake_accumulator_;
      if (!superstructure_queue.status->intake.estopped) {
        new_superstructure_goal->intake.voltage_rollers = 10.0;
      }
    }

    if (data.IsPressed(kHang)) {
      new_superstructure_goal->intake.voltage_rollers = -10.0;
      new_superstructure_goal->intake.disable_intake = true;
    } else if (data.IsPressed(kIntakeIn) || data.IsPressed(kFire)) {
      if (robot_velocity_ > 2.0) {
        new_superstructure_goal->intake.voltage_rollers = 12.0;
      } else {
        new_superstructure_goal->intake.voltage_rollers = 11.0;
      }
    } else if (data.IsPressed(kIntakeOut)) {
      new_superstructure_goal->intake.voltage_rollers = -8.0;
    }
    if (intake_goal_ < 0.1) {
      new_superstructure_goal->intake.voltage_rollers =
          ::std::min(8.0, new_superstructure_goal->intake.voltage_rollers);
    }

    if (data.IsPressed(kReverseIndexer)) {
      new_superstructure_goal->indexer.voltage_rollers = -12.0;
      new_superstructure_goal->indexer.angular_velocity = 4.0;
      new_superstructure_goal->indexer.angular_velocity = 1.0;
    } else if (fire_) {
      new_superstructure_goal->indexer.voltage_rollers = 12.0;
      switch (last_shot_distance_)  {
        case ShotDistance::VISION_SHOT:
          new_superstructure_goal->indexer.angular_velocity = -1.25 * M_PI;
          break;
        case ShotDistance::MIDDLE_SHOT:
        case ShotDistance::FAR_SHOT:
          new_superstructure_goal->indexer.angular_velocity = -2.25 * M_PI;
          break;
      }
    } else {
      new_superstructure_goal->indexer.voltage_rollers = 0.0;
      new_superstructure_goal->indexer.angular_velocity = 0.0;
    }

    LOG_STRUCT(DEBUG, "sending goal", *new_superstructure_goal);
    if (!new_superstructure_goal.Send()) {
      LOG(ERROR, "Sending superstructure goal failed.\n");
    }
  }

 private:
  void StartAuto() {
    LOG(INFO, "Starting auto mode\n");

    ::frc971::autonomous::AutonomousActionParams params;
    ::frc971::autonomous::auto_mode.FetchLatest();
    if (::frc971::autonomous::auto_mode.get() != nullptr) {
      params.mode = ::frc971::autonomous::auto_mode->mode;
    } else {
      LOG(WARNING, "no auto mode values\n");
      params.mode = 0;
    }
    action_queue_.EnqueueAction(autonomous_action_factory_.Make(params));
  }

  void StopAuto() {
    LOG(INFO, "Stopping auto mode\n");
    action_queue_.CancelAllActions();
  }

  // Current goals to send to the robot.
  double intake_goal_ = 0.0;
  double turret_goal_ = 0.0;
  double hood_goal_ = 0.3;
  double shooter_velocity_ = 0.0;

  bool was_running_ = false;
  bool auto_running_ = false;

  bool vision_distance_shot_ = false;

  bool fire_ = false;
  double robot_velocity_ = 0.0;

  ::aos::common::actions::ActionQueue action_queue_;

  ::frc971::autonomous::BaseAutonomousActor::Factory autonomous_action_factory_;
};

}  // namespace joysticks
}  // namespace input
}  // namespace y2017

int main() {
  ::aos::Init(-1);
  ::aos::ShmEventLoop event_loop;
  ::y2017::input::joysticks::Reader reader(&event_loop);
  reader.Run();
  ::aos::Cleanup();
}
