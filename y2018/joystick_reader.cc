#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <mutex>

#include "aos/common/actions/actions.h"
#include "aos/common/input/driver_station_data.h"
#include "aos/common/logging/logging.h"
#include "aos/common/stl_mutex.h"
#include "aos/common/time.h"
#include "aos/common/util/log_interval.h"
#include "aos/input/drivetrain_input.h"
#include "aos/input/joystick_input.h"
#include "aos/linux_code/init.h"
#include "aos/vision/events/udp.h"
#include "frc971/autonomous/auto.q.h"
#include "frc971/autonomous/base_autonomous_actor.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "y2018/control_loops/drivetrain/drivetrain_base.h"
#include "y2018/control_loops/superstructure/arm/generated_graph.h"
#include "y2018/control_loops/superstructure/superstructure.q.h"
#include "y2018/vision.pb.h"

using ::aos::monotonic_clock;
using ::frc971::control_loops::drivetrain_queue;
using ::y2018::control_loops::superstructure_queue;
using ::aos::events::ProtoTXUdpSocket;
using ::aos::events::RXUdpSocket;
using ::aos::input::driver_station::ButtonLocation;
using ::aos::input::driver_station::ControlBit;
using ::aos::input::driver_station::JoystickAxis;
using ::aos::input::driver_station::POVLocation;
using ::aos::input::DrivetrainInputReader;

using ::y2018::control_loops::superstructure::arm::FrontPoints;
using ::y2018::control_loops::superstructure::arm::BackPoints;

namespace y2018 {
namespace input {
namespace joysticks {

namespace arm = ::y2018::control_loops::superstructure::arm;

const ButtonLocation kIntakeClosed(3, 2);
const ButtonLocation kDuck(3, 9);
const ButtonLocation kSmallBox(3, 1);

const ButtonLocation kIntakeIn(3, 4);
const ButtonLocation kIntakeOut(3, 3);

const ButtonLocation kArmFrontHighBox(4, 11);
const ButtonLocation kArmFrontExtraHighBox(4, 1);
const ButtonLocation kArmFrontMiddle2Box(4, 9);
const ButtonLocation kArmFrontMiddle1Box(4, 7);
const ButtonLocation kArmFrontLowBox(4, 5);
const ButtonLocation kArmFrontSwitch(3, 7);

const ButtonLocation kArmBackHighBox(4, 12);
const ButtonLocation kArmBackExtraHighBox(3, 14);
const ButtonLocation kArmBackMiddle2Box(4, 10);
const ButtonLocation kArmBackMiddle1Box(4, 8);
const ButtonLocation kArmBackLowBox(4, 6);
const ButtonLocation kArmBackSwitch(3, 10);

const ButtonLocation kArmAboveHang(3, 15);
const ButtonLocation kArmBelowHang(3, 16);

const ButtonLocation kWinch(4, 2);

const ButtonLocation kArmNeutral(3, 8);
const ButtonLocation kArmUp(3, 11);

const ButtonLocation kArmStepUp(3, 13);
const ButtonLocation kArmStepDown(3, 12);

const ButtonLocation kArmPickupBoxFromIntake(4, 3);

const ButtonLocation kClawOpen(4, 4);
const ButtonLocation kDriverClawOpen(2, 4);

std::unique_ptr<DrivetrainInputReader> drivetrain_input_reader_;

class Reader : public ::aos::input::JoystickInput {
 public:
  Reader() : video_tx_("10.9.71.179", 5000) {
    drivetrain_input_reader_ = DrivetrainInputReader::Make(
        DrivetrainInputReader::InputType::kPistol,
        ::y2018::control_loops::drivetrain::GetDrivetrainConfig());
  }

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

  void HandleDrivetrain(const ::aos::input::driver_station::Data &data) {
    drivetrain_input_reader_->HandleDrivetrain(data);
  }

  void HandleTeleop(const ::aos::input::driver_station::Data &data) {
    if (!data.GetControlBit(ControlBit::kEnabled)) {
      action_queue_.CancelAllActions();
      LOG(DEBUG, "Canceling\n");
    }

    superstructure_queue.position.FetchLatest();
    superstructure_queue.status.FetchLatest();
    if (!superstructure_queue.status.get() ||
        !superstructure_queue.position.get()) {
      LOG(ERROR, "Got no superstructure status packet.\n");
      return;
    }

    auto new_superstructure_goal = superstructure_queue.goal.MakeMessage();

    new_superstructure_goal->intake.left_intake_angle = intake_goal_;
    new_superstructure_goal->intake.right_intake_angle = intake_goal_;

    if (data.PosEdge(kIntakeIn) || data.PosEdge(kSmallBox) ||
        data.PosEdge(kIntakeClosed)) {
      vision_control_.set_high_video(false);
    }

    if (data.IsPressed(kIntakeIn)) {
      // Turn on the rollers.
      new_superstructure_goal->intake.roller_voltage = 8.0;
    } else if (data.IsPressed(kIntakeOut)) {
      // Turn off the rollers.
      new_superstructure_goal->intake.roller_voltage = -12.0;
    } else {
      // We don't want the rollers on.
      new_superstructure_goal->intake.roller_voltage = 0.0;
    }

    if (data.IsPressed(kSmallBox)) {
      // Deploy the intake.
      if (superstructure_queue.position->box_back_beambreak_triggered) {
        intake_goal_ = 0.30;
      } else {
        if (new_superstructure_goal->intake.roller_voltage > 0.1 &&
            superstructure_queue.position->box_distance < 0.15) {
          intake_goal_ = 0.18;
        } else {
          intake_goal_ = -0.60;
        }
      }
    } else if (data.IsPressed(kIntakeClosed)) {
      // Deploy the intake.
      if (superstructure_queue.position->box_back_beambreak_triggered) {
        intake_goal_ = 0.30;
      } else {
        if (new_superstructure_goal->intake.roller_voltage > 0.1) {
          if (superstructure_queue.position->box_distance < 0.10) {
            new_superstructure_goal->intake.roller_voltage -= 3.0;
            intake_goal_ = 0.22;
          } else if (superstructure_queue.position->box_distance < 0.17) {
            intake_goal_ = 0.13;
          } else if (superstructure_queue.position->box_distance < 0.25) {
            intake_goal_ = 0.05;
          } else {
            intake_goal_ = -0.10;
          }
          if (drivetrain_input_reader_->robot_velocity() < -0.1 &&
              superstructure_queue.position->box_distance > 0.15) {
            intake_goal_ += 0.10;
          }
        } else {
          intake_goal_ = -0.60;
        }
      }
    } else {
      // Bring in the intake.
      intake_goal_ = -3.20;
    }

    if (new_superstructure_goal->intake.roller_voltage > 0.1 &&
        intake_goal_ > 0.0) {
      if (superstructure_queue.position->box_distance < 0.10) {
        new_superstructure_goal->intake.roller_voltage -= 3.0;
      }
      new_superstructure_goal->intake.roller_voltage += 3.0;
    }

    // If we are disabled, stay at the node closest to where we start.  This
    // should remove long motions when enabled.
    if (!data.GetControlBit(ControlBit::kEnabled) || never_disabled_) {
      arm_goal_position_ = superstructure_queue.status->arm.current_node;
      never_disabled_ = false;
    }

    bool grab_box = false;
    if (data.IsPressed(kArmPickupBoxFromIntake)) {
      grab_box = true;
    }
    const bool near_goal =
        superstructure_queue.status->arm.current_node == arm_goal_position_ &&
        superstructure_queue.status->arm.path_distance_to_go < 1e-3;
    if (data.IsPressed(kArmStepDown) && near_goal) {
      uint32_t *front_point = ::std::find(
          front_points_.begin(), front_points_.end(), arm_goal_position_);
      uint32_t *back_point = ::std::find(
          back_points_.begin(), back_points_.end(), arm_goal_position_);
      if (front_point != front_points_.end()) {
        ++front_point;
        if (front_point != front_points_.end()) {
          arm_goal_position_ = *front_point;
        }
      } else if (back_point != back_points_.end()) {
        ++back_point;
        if (back_point != back_points_.end()) {
          arm_goal_position_ = *back_point;
        }
      }
    } else if (data.IsPressed(kArmStepUp) && near_goal) {
      const uint32_t *front_point = ::std::find(
          front_points_.begin(), front_points_.end(), arm_goal_position_);
      const uint32_t *back_point = ::std::find(
          back_points_.begin(), back_points_.end(), arm_goal_position_);
      if (front_point != front_points_.end()) {
        if (front_point != front_points_.begin()) {
          --front_point;
          arm_goal_position_ = *front_point;
        }
      } else if (back_point != back_points_.end()) {
        if (back_point != back_points_.begin()) {
          --back_point;
          arm_goal_position_ = *back_point;
        }
      }
    } else if (data.PosEdge(kArmPickupBoxFromIntake)) {
      vision_control_.set_high_video(false);
      arm_goal_position_ = arm::NeutralIndex();
    } else if (data.IsPressed(kDuck)) {
      vision_control_.set_high_video(false);
      arm_goal_position_ = arm::DuckIndex();
    } else if (data.IsPressed(kArmNeutral)) {
      vision_control_.set_high_video(false);
      arm_goal_position_ = arm::NeutralIndex();
    } else if (data.IsPressed(kArmUp)) {
      vision_control_.set_high_video(true);
      arm_goal_position_ = arm::UpIndex();
    } else if (data.IsPressed(kArmFrontSwitch)) {
      arm_goal_position_ = arm::FrontSwitchIndex();
    } else if (data.IsPressed(kArmFrontExtraHighBox)) {
      vision_control_.set_high_video(true);
      arm_goal_position_ = arm::FrontHighBoxIndex();
    } else if (data.IsPressed(kArmFrontHighBox)) {
      vision_control_.set_high_video(true);
      arm_goal_position_ = arm::FrontMiddle2BoxIndex();
    } else if (data.IsPressed(kArmFrontMiddle2Box)) {
      vision_control_.set_high_video(true);
      arm_goal_position_ = arm::FrontMiddle3BoxIndex();
    } else if (data.IsPressed(kArmFrontMiddle1Box)) {
      vision_control_.set_high_video(true);
      arm_goal_position_ = arm::FrontMiddle1BoxIndex();
    } else if (data.IsPressed(kArmFrontLowBox)) {
      vision_control_.set_high_video(true);
      arm_goal_position_ = arm::FrontLowBoxIndex();
    } else if (data.IsPressed(kArmBackExtraHighBox)) {
      arm_goal_position_ = arm::BackHighBoxIndex();
    } else if (data.IsPressed(kArmBackHighBox) ||
               data.IsPressed(kArmBackMiddle2Box)) {
      arm_goal_position_ = arm::BackMiddle2BoxIndex();
    } else if (data.IsPressed(kArmBackMiddle1Box)) {
      arm_goal_position_ = arm::BackMiddle1BoxIndex();
    } else if (data.IsPressed(kArmBackLowBox)) {
      arm_goal_position_ = arm::BackLowBoxIndex();
    }  else if (data.IsPressed(kArmBackSwitch)) {
      arm_goal_position_ = arm::BackSwitchIndex();
    } else if (data.IsPressed(kArmAboveHang)) {
      if (data.IsPressed(kIntakeIn)) {
        arm_goal_position_ = arm::SelfHangIndex();
      } else if (data.IsPressed(kIntakeOut)) {
        arm_goal_position_ = arm::PartnerHangIndex();
      } else {
        arm_goal_position_ = arm::AboveHangIndex();
      }
    } else if (data.IsPressed(kArmBelowHang)) {
      arm_goal_position_ = arm::BelowHangIndex();
    }

    new_superstructure_goal->deploy_fork =
        data.IsPressed(kArmAboveHang) && data.IsPressed(kClawOpen);

    if (new_superstructure_goal->deploy_fork) {
      intake_goal_ = -2.0;
    }

    if (data.IsPressed(kWinch)) {
      LOG(INFO, "Winching\n");
      new_superstructure_goal->voltage_winch = 12.0;
    } else {
      new_superstructure_goal->voltage_winch = 0.0;
    }

    new_superstructure_goal->hook_release = data.IsPressed(kArmBelowHang);

    new_superstructure_goal->arm_goal_position = arm_goal_position_;

    if ((data.IsPressed(kClawOpen) && data.IsPressed(kDriverClawOpen)) ||
        data.PosEdge(kArmPickupBoxFromIntake)) {
      new_superstructure_goal->open_claw = true;
    } else {
      new_superstructure_goal->open_claw = false;
    }

    new_superstructure_goal->grab_box = grab_box;

    LOG_STRUCT(DEBUG, "sending goal", *new_superstructure_goal);
    if (!new_superstructure_goal.Send()) {
      LOG(ERROR, "Sending superstructure goal failed.\n");
    }

    video_tx_.Send(vision_control_);
  }

 private:
  void StartAuto() {
    LOG(INFO, "Starting auto mode\n");

    ::frc971::autonomous::AutonomousActionParams params;
    ::frc971::autonomous::auto_mode.FetchLatest();
    if (::frc971::autonomous::auto_mode.get() != nullptr) {
      params.mode = ::frc971::autonomous::auto_mode->mode << 2;
    } else {
      LOG(WARNING, "no auto mode values\n");
      params.mode = 0;
    }
    // TODO(austin): use the mode later if we care.  We don't care right now.
    params.mode = static_cast<int>(::aos::joystick_state->switch_left) |
                  (static_cast<int>(::aos::joystick_state->scale_left) << 1);
    action_queue_.EnqueueAction(
        ::frc971::autonomous::MakeAutonomousAction(params));
  }

  void StopAuto() {
    LOG(INFO, "Stopping auto mode\n");
    action_queue_.CancelAllActions();
  }

  // Current goals to send to the robot.
  double intake_goal_ = 0.0;

  bool was_running_ = false;
  bool auto_running_ = false;
  bool never_disabled_ = true;

  uint32_t arm_goal_position_ = 0;
  VisionControl vision_control_;

  decltype(FrontPoints()) front_points_ = FrontPoints();
  decltype(BackPoints()) back_points_ = BackPoints();

  ::aos::common::actions::ActionQueue action_queue_;

  ProtoTXUdpSocket<VisionControl> video_tx_;
};

}  // namespace joysticks
}  // namespace input
}  // namespace y2018

int main() {
  ::aos::Init(-1);
  ::y2018::input::joysticks::Reader reader;
  reader.Run();
  ::aos::Cleanup();
}
