#include "y2019/control_loops/superstructure/superstructure.h"

#include "aos/controls/control_loops.q.h"
#include "aos/events/event-loop.h"
#include "frc971/control_loops/control_loops.q.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/control_loops/static_zeroing_single_dof_profiled_subsystem.h"
#include "y2019/status_light.q.h"

namespace y2019 {
namespace control_loops {
namespace superstructure {

Superstructure::Superstructure(::aos::EventLoop *event_loop,
                               const ::std::string &name)
    : aos::controls::ControlLoop<SuperstructureQueue>(event_loop, name),
      status_light_sender_(
          event_loop->MakeSender<::y2019::StatusLight>(".y2019.status_light")),
      drivetrain_status_fetcher_(
          event_loop
              ->MakeFetcher<::frc971::control_loops::DrivetrainQueue::Status>(
                  ".frc971.control_loops.drivetrain_queue.status")),
      elevator_(constants::GetValues().elevator.subsystem_params),
      wrist_(constants::GetValues().wrist.subsystem_params),
      intake_(constants::GetValues().intake),
      stilts_(constants::GetValues().stilts.subsystem_params) {}

void Superstructure::RunIteration(const SuperstructureQueue::Goal *unsafe_goal,
                                  const SuperstructureQueue::Position *position,
                                  SuperstructureQueue::Output *output,
                                  SuperstructureQueue::Status *status) {
  if (WasReset()) {
    AOS_LOG(ERROR, "WPILib reset, restarting\n");
    elevator_.Reset();
    wrist_.Reset();
    intake_.Reset();
    stilts_.Reset();
  }

  elevator_.Iterate(unsafe_goal != nullptr ? &(unsafe_goal->elevator) : nullptr,
                    &(position->elevator),
                    output != nullptr ? &(output->elevator_voltage) : nullptr,
                    &(status->elevator));

  wrist_.Iterate(unsafe_goal != nullptr ? &(unsafe_goal->wrist) : nullptr,
                 &(position->wrist),
                 output != nullptr ? &(output->wrist_voltage) : nullptr,
                 &(status->wrist));

  intake_.Iterate(unsafe_goal != nullptr ? &(unsafe_goal->intake) : nullptr,
                  &(position->intake_joint),
                  output != nullptr ? &(output->intake_joint_voltage) : nullptr,
                  &(status->intake));

  stilts_.Iterate(unsafe_goal != nullptr ? &(unsafe_goal->stilts) : nullptr,
                  &(position->stilts),
                  output != nullptr ? &(output->stilts_voltage) : nullptr,
                  &(status->stilts));

  vacuum_.Iterate(unsafe_goal != nullptr ? &(unsafe_goal->suction) : nullptr,
                  position->suction_pressure, output, &(status->has_piece),
                  event_loop());

  status->zeroed = status->elevator.zeroed && status->wrist.zeroed &&
                   status->intake.zeroed && status->stilts.zeroed;

  status->estopped = status->elevator.estopped || status->wrist.estopped ||
                     status->intake.estopped || status->stilts.estopped;

  if (output) {
    if (unsafe_goal && status->intake.position > kMinIntakeAngleForRollers) {
      output->intake_roller_voltage = unsafe_goal->roller_voltage;
    } else {
      output->intake_roller_voltage = 0.0;
    }
  }

  if (unsafe_goal) {
    if (!unsafe_goal->suction.grab_piece) {
      wrist_.set_controller_index(0);
      elevator_.set_controller_index(0);
    } else if (unsafe_goal->suction.gamepiece_mode == 0) {
      wrist_.set_controller_index(1);
      elevator_.set_controller_index(1);
    } else {
      wrist_.set_controller_index(2);
      elevator_.set_controller_index(2);
    }
  }

  // TODO(theo) move these up when Iterate() is split
  // update the goals
  collision_avoidance_.UpdateGoal(status, unsafe_goal);

  elevator_.set_min_position(collision_avoidance_.min_elevator_goal());
  wrist_.set_min_position(collision_avoidance_.min_wrist_goal());
  wrist_.set_max_position(collision_avoidance_.max_wrist_goal());
  intake_.set_min_position(collision_avoidance_.min_intake_goal());
  intake_.set_max_position(collision_avoidance_.max_intake_goal());

  drivetrain_status_fetcher_.Fetch();

  if (status && unsafe_goal) {
    // Light Logic
    if (status->estopped) {
      // Estop is red
      SendColors(1.0, 0.0, 0.0);
    } else if (drivetrain_status_fetcher_.get() &&
               drivetrain_status_fetcher_->line_follow_logging.frozen) {
      // Vision align is flashing white for button pressed, purple for target
      // acquired.
      ++line_blink_count_;
      if (line_blink_count_ < 20) {
        if (drivetrain_status_fetcher_->line_follow_logging.have_target) {
          SendColors(1.0, 0.0, 1.0);
        } else {
          SendColors(1.0, 1.0, 1.0);
        }
      } else {
        // And then flash with green if we have a game piece.
        if (status->has_piece) {
          SendColors(0.0, 1.0, 0.0);
        } else {
          SendColors(0.0, 0.0, 0.0);
        }
      }

      if (line_blink_count_ > 40) {
        line_blink_count_ = 0;
      }
    } else {
      line_blink_count_ = 0;
      if (status->has_piece) {
        // Green if we have a game piece.
        SendColors(0.0, 1.0, 0.0);
      } else if (unsafe_goal->suction.gamepiece_mode == 0 &&
                 !status->has_piece) {
        // Ball mode is orange
        SendColors(1.0, 0.1, 0.0);
      } else if (unsafe_goal->suction.gamepiece_mode == 1 &&
                 !status->has_piece) {
        // Disk mode is deep blue
        SendColors(0.05, 0.1, 0.5);
      } else {
        SendColors(0.0, 0.0, 0.0);
      }
    }
  }
}

void Superstructure::SendColors(float red, float green, float blue) {
  auto new_status_light = status_light_sender_.MakeMessage();
  new_status_light->red = red;
  new_status_light->green = green;
  new_status_light->blue = blue;

  if (!new_status_light.Send()) {
    AOS_LOG(ERROR, "Failed to send lights.\n");
  }
}

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2019
