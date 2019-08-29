#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <mutex>
#include <google/protobuf/stubs/stringprintf.h>

#include "aos/actions/actions.h"
#include "aos/init.h"
#include "aos/input/action_joystick_input.h"
#include "aos/input/driver_station_data.h"
#include "aos/input/drivetrain_input.h"
#include "aos/logging/logging.h"
#include "aos/network/team_number.h"
#include "aos/stl_mutex/stl_mutex.h"
#include "aos/time/time.h"
#include "aos/util/log_interval.h"
#include "aos/vision/events/udp.h"
#include "frc971/autonomous/base_autonomous_actor.h"
#include "y2018/control_loops/drivetrain/drivetrain_base.h"
#include "y2018/control_loops/superstructure/arm/generated_graph.h"
#include "y2018/control_loops/superstructure/superstructure_goal_generated.h"
#include "y2018/control_loops/superstructure/superstructure_position_generated.h"
#include "y2018/control_loops/superstructure/superstructure_status_generated.h"
#include "y2018/vision.pb.h"

using ::aos::monotonic_clock;
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
using google::protobuf::StringPrintf;

const ButtonLocation kIntakeClosed(3, 2);
const ButtonLocation kDuck(3, 9);
const ButtonLocation kSmallBox(3, 1);

const ButtonLocation kIntakeIn(3, 4);
const ButtonLocation kIntakeOut(3, 3);

const ButtonLocation kArmBackHighBox(4, 11);
const ButtonLocation kArmBackExtraHighBox(4, 1);
const ButtonLocation kArmBackMiddle2Box(4, 9);
const ButtonLocation kArmBackMiddle1Box(4, 7);
const ButtonLocation kArmBackLowBox(4, 5);
const ButtonLocation kArmBackSwitch(3, 7);

const ButtonLocation kArmFrontHighBox(4, 12);
const ButtonLocation kArmFrontExtraHighBox(3, 14);
const ButtonLocation kArmFrontMiddle2Box(4, 10);
const ButtonLocation kArmFrontMiddle1Box(4, 8);
const ButtonLocation kArmFrontLowBox(4, 6);
const ButtonLocation kArmFrontSwitch(3, 10);

const ButtonLocation kArmAboveHang(3, 15);
const ButtonLocation kArmBelowHang(3, 16);

const ButtonLocation kEmergencyUp(3, 5);
const ButtonLocation kEmergencyDown(3, 6);

const ButtonLocation kWinch(4, 2);

const ButtonLocation kArmNeutral(3, 8);
const ButtonLocation kArmUp(3, 11);

const ButtonLocation kArmStepUp(3, 13);
const ButtonLocation kArmStepDown(3, 12);

const ButtonLocation kArmPickupBoxFromIntake(4, 3);

const ButtonLocation kClawOpen(4, 4);
const ButtonLocation kDriverClawOpen(2, 4);

class Reader : public ::aos::input::ActionJoystickInput {
 public:
  Reader(::aos::EventLoop *event_loop)
      : ::aos::input::ActionJoystickInput(
            event_loop,
            ::y2018::control_loops::drivetrain::GetDrivetrainConfig(),
            ::aos::network::GetTeamNumber() == 971
                ? DrivetrainInputReader::InputType::kPistol
                : DrivetrainInputReader::InputType::kSteeringWheel,
            {}),
        superstructure_position_fetcher_(
            event_loop->MakeFetcher<
                ::y2018::control_loops::superstructure::Position>(
                ".frc971.control_loops.superstructure_queue.position")),
        superstructure_status_fetcher_(
            event_loop->MakeFetcher<
                ::y2018::control_loops::superstructure::Status>(
                ".frc971.control_loops.superstructure_queue.status")),
        superstructure_goal_sender_(
            event_loop
                ->MakeSender<::y2018::control_loops::superstructure::Goal>(
                    ".frc971.control_loops.superstructure_queue.goal")) {
    const uint16_t team = ::aos::network::GetTeamNumber();

    video_tx_.reset(new ProtoTXUdpSocket<VisionControl>(
        StringPrintf("10.%d.%d.179", team / 100, team % 100), 5000));
  }

  void HandleTeleop(const ::aos::input::driver_station::Data &data) override {
    superstructure_position_fetcher_.Fetch();
    superstructure_status_fetcher_.Fetch();
    if (!superstructure_status_fetcher_.get() ||
        !superstructure_position_fetcher_.get()) {
      AOS_LOG(ERROR, "Got no superstructure status packet.\n");
      return;
    }

    auto builder = superstructure_goal_sender_.MakeBuilder();

    double roller_voltage = 0.0;
    bool trajectory_override = false;

    if (data.PosEdge(kIntakeIn) || data.PosEdge(kSmallBox) ||
        data.PosEdge(kIntakeClosed)) {
      vision_control_.set_high_video(false);
    }

    if (data.IsPressed(kIntakeIn)) {
      // Turn on the rollers.
      roller_voltage = 8.0;
    } else if (data.IsPressed(kIntakeOut)) {
      // Turn off the rollers.
      roller_voltage = -12.0;
    } else {
      // We don't want the rollers on.
      roller_voltage = 0.0;
    }

    if (data.IsPressed(kSmallBox)) {
      // Deploy the intake.
      if (superstructure_position_fetcher_->box_back_beambreak_triggered()) {
        intake_goal_ = 0.30;
      } else {
        if (roller_voltage > 0.1 &&
            superstructure_position_fetcher_->box_distance() < 0.15) {
          intake_goal_ = 0.18;
        } else {
          intake_goal_ = -0.60;
        }
      }
    } else if (data.IsPressed(kIntakeClosed)) {
      // Deploy the intake.
      if (superstructure_position_fetcher_->box_back_beambreak_triggered()) {
        intake_goal_ = 0.30;
      } else {
        if (roller_voltage > 0.1) {
          if (superstructure_position_fetcher_->box_distance() < 0.10) {
            roller_voltage -= 3.0;
            intake_goal_ = 0.22;
          } else if (superstructure_position_fetcher_->box_distance() < 0.17) {
            intake_goal_ = 0.13;
          } else if (superstructure_position_fetcher_->box_distance() < 0.25) {
            intake_goal_ = 0.05;
          } else {
            intake_goal_ = -0.10;
          }
          if (robot_velocity() < -0.1 &&
              superstructure_position_fetcher_->box_distance() > 0.15) {
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

    if (roller_voltage > 0.1 &&
        intake_goal_ > 0.0) {
      if (superstructure_position_fetcher_->box_distance() < 0.10) {
        roller_voltage -= 3.0;
      }
      roller_voltage += 3.0;
    }

    // If we are disabled, stay at the node closest to where we start.  This
    // should remove long motions when enabled.
    if (!data.GetControlBit(ControlBit::kEnabled) || never_disabled_) {
      arm_goal_position_ =
          superstructure_status_fetcher_->arm()->current_node();
      never_disabled_ = false;
    }

    bool grab_box = false;
    if (data.IsPressed(kArmPickupBoxFromIntake)) {
      grab_box = true;
    }
    const bool near_goal =
        superstructure_status_fetcher_->arm()->current_node() ==
            arm_goal_position_ &&
        superstructure_status_fetcher_->arm()->path_distance_to_go() < 1e-3;
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
    } else if (data.IsPressed(kArmBackSwitch)) {
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

    if (data.IsPressed(kEmergencyDown)) {
      arm_goal_position_ = arm::NeutralIndex();
      trajectory_override = true;
    } else if (data.IsPressed(kEmergencyUp)) {
      arm_goal_position_ = arm::UpIndex();
      trajectory_override = true;
    } else {
      trajectory_override = false;
    }

    const bool deploy_fork =
        data.IsPressed(kArmAboveHang) && data.IsPressed(kClawOpen);

    if (deploy_fork) {
      intake_goal_ = -2.0;
    }

    control_loops::superstructure::IntakeGoal::Builder intake_goal_builder =
        builder.MakeBuilder<control_loops::superstructure::IntakeGoal>();

    intake_goal_builder.add_left_intake_angle(intake_goal_);
    intake_goal_builder.add_right_intake_angle(intake_goal_);
    intake_goal_builder.add_roller_voltage(roller_voltage);

    flatbuffers::Offset<control_loops::superstructure::IntakeGoal>
        intake_goal_offset = intake_goal_builder.Finish();

    control_loops::superstructure::Goal::Builder superstructure_builder =
        builder.MakeBuilder<control_loops::superstructure::Goal>();

    superstructure_builder.add_intake(intake_goal_offset);
    superstructure_builder.add_grab_box(grab_box);
    superstructure_builder.add_trajectory_override(trajectory_override);
    superstructure_builder.add_deploy_fork(deploy_fork);

    if (data.IsPressed(kWinch)) {
      AOS_LOG(INFO, "Winching\n");
      superstructure_builder.add_voltage_winch(12.0);
    } else {
      superstructure_builder.add_voltage_winch(0.0);
    }

    superstructure_builder.add_hook_release(data.IsPressed(kArmBelowHang));

    superstructure_builder.add_arm_goal_position(arm_goal_position_);
    if (data.IsPressed(kArmFrontSwitch) || data.IsPressed(kArmBackSwitch)) {
      superstructure_builder.add_open_threshold(0.35);
    } else {
      superstructure_builder.add_open_threshold(0.0);
    }

    if ((data.IsPressed(kClawOpen) && data.IsPressed(kDriverClawOpen)) ||
        data.PosEdge(kArmPickupBoxFromIntake) ||
        (data.IsPressed(kClawOpen) &&
         (data.IsPressed(kArmFrontSwitch) || data.IsPressed(kArmBackSwitch)))) {
      superstructure_builder.add_open_claw(true);
    } else {
      superstructure_builder.add_open_claw(false);
    }

    if (!builder.Send(superstructure_builder.Finish())) {
      AOS_LOG(ERROR, "Sending superstructure goal failed.\n");
    }

    video_tx_->Send(vision_control_);
  }

 private:
  uint32_t GetAutonomousMode() override {
    // Low bit is switch, high bit is scale.  1 means left, 0 means right.
    return mode();
  }

  ::aos::Fetcher<control_loops::superstructure::Position>
      superstructure_position_fetcher_;
  ::aos::Fetcher<control_loops::superstructure::Status>
      superstructure_status_fetcher_;
  ::aos::Sender<control_loops::superstructure::Goal>
      superstructure_goal_sender_;

  // Current goals to send to the robot.
  double intake_goal_ = 0.0;

  bool never_disabled_ = true;

  uint32_t arm_goal_position_ = 0;
  VisionControl vision_control_;

  decltype(FrontPoints()) front_points_ = FrontPoints();
  decltype(BackPoints()) back_points_ = BackPoints();

  ::std::unique_ptr<ProtoTXUdpSocket<VisionControl>> video_tx_;
};

}  // namespace joysticks
}  // namespace input
}  // namespace y2018

int main() {
  ::aos::InitNRT(true);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("config.json");

  ::aos::ShmEventLoop event_loop(&config.message());
  ::y2018::input::joysticks::Reader reader(&event_loop);

  event_loop.Run();

  ::aos::Cleanup();
}
