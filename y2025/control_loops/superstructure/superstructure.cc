#include "y2025/control_loops/superstructure/superstructure.h"

#include <chrono>

#include "aos/events/event_loop.h"
#include "aos/flatbuffer_merge.h"
#include "aos/network/team_number.h"
#include "aos/time/time.h"
#include "frc971/zeroing/wrap.h"

ABSL_FLAG(bool, ignore_distance, false,
          "If true, ignore distance when shooting and obey joystick_reader");

namespace y2025::control_loops::superstructure {

using ::aos::monotonic_clock;

using frc971::control_loops::AbsoluteEncoderProfiledJointStatus;
using frc971::control_loops::PotAndAbsoluteEncoderProfiledJointStatus;
using frc971::control_loops::RelativeEncoderProfiledJointStatus;

Superstructure::Superstructure(::aos::EventLoop *event_loop,
                               const ::std::string &name)
    : frc971::controls::ControlLoop<Goal, Position, Status, Output>(event_loop,
                                                                    name),
      constants_fetcher_(event_loop),
      robot_constants_(&constants_fetcher_.constants()),
      joystick_state_fetcher_(
          event_loop->MakeFetcher<aos::JoystickState>("/aos")),
      auto_superstructure_goal_fetcher_(
          event_loop->MakeFetcher<y2025::control_loops::superstructure::Goal>(
              "/imu/autonomous")),
      elevator_(
          robot_constants_->common()->elevator(),
          robot_constants_->robot()->elevator_constants()->zeroing_constants()),
      pivot_(robot_constants_->common()->pivot(),
             robot_constants_->robot()->pivot_constants()->zeroing_constants()),
      wrist_(robot_constants_->common()->wrist(),
             robot_constants_->robot()->wrist_constants()->zeroing_constants()),
      rio_can_position_fetcher_(
          event_loop->MakeFetcher<CANPosition>("/superstructure/rio")) {
  event_loop->SetRuntimeRealtimePriority(30);
}

bool Superstructure::GetIntakeComplete() { return intake_complete_; }

void Superstructure::RunIteration(const Goal *unsafe_goal,
                                  const Position *position,
                                  aos::Sender<Output>::Builder *output,
                                  aos::Sender<Status>::Builder *status) {
  if (WasReset()) {
    AOS_LOG(ERROR, "WPILib reset, restarting\n");
    elevator_.Reset();
    pivot_.Reset();
    wrist_.Reset();
  }
  OutputT output_struct;

  joystick_state_fetcher_.Fetch();
  if (joystick_state_fetcher_.get() != nullptr &&
      joystick_state_fetcher_->has_alliance()) {
    alliance_ = joystick_state_fetcher_->alliance();
  }

  if (rio_can_position_fetcher_.Fetch()) {
    intake_complete_ =
        rio_can_position_fetcher_.get()->end_effector()->torque_current() >
        kEndEffectorMotorTorqueThreshold;
  }
  if (joystick_state_fetcher_.get() != nullptr &&
      joystick_state_fetcher_->autonomous() &&
      auto_superstructure_goal_fetcher_.Fetch()) {
    unsafe_goal = auto_superstructure_goal_fetcher_.get();
  }

  double end_effector_voltage =
      robot_constants_->common()->end_effector_idling_voltage();
  EndEffectorStatus end_effector_status = EndEffectorStatus::NEUTRAL;

  if (unsafe_goal != nullptr) {
    switch (unsafe_goal->end_effector_goal()) {
      case EndEffectorGoal::NEUTRAL:
        break;
      case EndEffectorGoal::INTAKE:
        if (!intake_complete_) {
          end_effector_status = EndEffectorStatus::INTAKE;
          end_effector_voltage =
              robot_constants_->common()->end_effector_voltages()->intake();
        } else {
          end_effector_status = EndEffectorStatus::NEUTRAL;
          end_effector_voltage =
              robot_constants_->common()->end_effector_idling_voltage();
        }
        break;
      case EndEffectorGoal::SPIT:
        end_effector_status = EndEffectorStatus::SPITTING;
        if (unsafe_goal->has_elevator_goal() &&
            unsafe_goal->elevator_goal() == ElevatorGoal::SCORE_L1) {
          end_effector_voltage =
              robot_constants_->common()->end_effector_voltages()->spit_l1();
        } else {
          end_effector_voltage =
              robot_constants_->common()->end_effector_voltages()->spit();
        }
        intake_complete_ = false;

        break;
    }
  }

  output_struct.end_effector_voltage = end_effector_voltage;

  aos::fbs::FixedStackAllocator<aos::fbs::Builder<
      frc971::control_loops::
          StaticZeroingSingleDOFProfiledSubsystemGoalStatic>::kBufferSize>
      elevator_goal_buffer;

  aos::fbs::Builder<
      frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoalStatic>
      elevator_goal_builder(&elevator_goal_buffer);

  double elevator_position =
      robot_constants_->common()->elevator_set_points()->neutral();
  bool pivot_can_move_ = true;
  bool wrist_can_move_ = true;

  if (unsafe_goal != nullptr) {
    switch (unsafe_goal->elevator_goal()) {
      case ElevatorGoal::NEUTRAL:
        break;
      case ElevatorGoal::INTAKE_HP:
        elevator_position =
            robot_constants_->common()->elevator_set_points()->intake_hp();
        break;
      case ElevatorGoal::INTAKE_GROUND:
        elevator_position =
            robot_constants_->common()->elevator_set_points()->intake_ground();
        break;
      case ElevatorGoal::SCORE_L1:
        elevator_position =
            robot_constants_->common()->elevator_set_points()->score_l1();
        break;
      case ElevatorGoal::SCORE_L2:
        elevator_position =
            robot_constants_->common()->elevator_set_points()->score_l2();
        break;
      case ElevatorGoal::SCORE_L3:
        elevator_position =
            robot_constants_->common()->elevator_set_points()->score_l3();
        break;
      case ElevatorGoal::SCORE_L4:
        elevator_position =
            robot_constants_->common()->elevator_set_points()->score_l4();
        break;
      case ElevatorGoal::ALGAE_L2:
        elevator_position =
            robot_constants_->common()->elevator_set_points()->algae_l2();
        break;
      case ElevatorGoal::ALGAE_L3:
        elevator_position =
            robot_constants_->common()->elevator_set_points()->algae_l3();
        break;
      case ElevatorGoal::BARGE:
        elevator_position =
            robot_constants_->common()->elevator_set_points()->barge();
        break;
      case ElevatorGoal::ALGAE_PROCESSOR:
        elevator_position = robot_constants_->common()
                                ->elevator_set_points()
                                ->algae_processor();
        break;
      case ElevatorGoal::ALGAE_GROUND:
        elevator_position =
            robot_constants_->common()->elevator_set_points()->algae_ground();
        break;
      case ElevatorGoal::INTAKE_HP_BACKUP:
        elevator_position = robot_constants_->common()
                                ->elevator_set_points()
                                ->intake_hp_backup();
        break;
      case ElevatorGoal::CLIMB:
        elevator_position =
            robot_constants_->common()->elevator_set_points()->climb();
        break;
    }

    // TODO: Generalize this logic to be done in a more generic way by being
    // able to decide per set-point the flow of actions between subsystems
    bool elevator_first_ =
        unsafe_goal->elevator_goal() == ElevatorGoal::SCORE_L4 ||
        unsafe_goal->elevator_goal() == ElevatorGoal::SCORE_L3;
    pivot_can_move_ =
        (!elevator_first_) ||
        (elevator_first_ &&
         Superstructure::PositionNear(
             elevator_.position(), elevator_position,
             robot_constants_->common()->pivot_can_move_elevator_threshold()));
    wrist_can_move_ =
        (!elevator_first_) ||
        (elevator_first_ &&
         PositionNear(
             elevator_.position(), elevator_position,
             robot_constants_->common()->wrist_can_move_elevator_threshold()));
  }

  PopulateStaticZeroingSingleDOFProfiledSubsystemGoal(
      elevator_goal_builder.get(), elevator_position);

  const frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoal
      *elevator_goal = &elevator_goal_builder->AsFlatbuffer();

  const flatbuffers::Offset<PotAndAbsoluteEncoderProfiledJointStatus>
      elevator_status_offset = elevator_.Iterate(
          elevator_goal, position->elevator(),
          output != nullptr ? &(output_struct.elevator_voltage) : nullptr,
          status->fbb());

  aos::fbs::FixedStackAllocator<aos::fbs::Builder<
      frc971::control_loops::
          StaticZeroingSingleDOFProfiledSubsystemGoalStatic>::kBufferSize>
      pivot_goal_buffer;

  aos::fbs::Builder<
      frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoalStatic>
      pivot_goal_builder(&pivot_goal_buffer);

  double pivot_position =
      robot_constants_->common()->pivot_set_points()->neutral();

  if (pivot_can_move_ && unsafe_goal != nullptr) {
    switch (unsafe_goal->pivot_goal()) {
      case PivotGoal::NEUTRAL:
        break;
      case PivotGoal::INTAKE_HP:
        pivot_position =
            robot_constants_->common()->pivot_set_points()->intake_hp();
        break;
      case PivotGoal::INTAKE_GROUND:
        pivot_position =
            robot_constants_->common()->pivot_set_points()->intake_ground();
        break;
      case PivotGoal::SCORE_L1:
        pivot_position =
            robot_constants_->common()->pivot_set_points()->score_l1();
        break;
      case PivotGoal::SCORE_L2:
        pivot_position =
            robot_constants_->common()->pivot_set_points()->score_l2();
        break;
      case PivotGoal::SCORE_L3:
        pivot_position =
            robot_constants_->common()->pivot_set_points()->score_l3();
        break;
      case PivotGoal::SCORE_L4:
        pivot_position =
            robot_constants_->common()->pivot_set_points()->score_l4();
        break;
      case PivotGoal::ALGAE_L2:
        pivot_position =
            robot_constants_->common()->pivot_set_points()->algae_l2();
        break;
      case PivotGoal::ALGAE_L3:
        pivot_position =
            robot_constants_->common()->pivot_set_points()->algae_l3();
        break;
      case PivotGoal::BARGE:
        pivot_position =
            robot_constants_->common()->pivot_set_points()->barge();
        break;
      case PivotGoal::ALGAE_PROCESSOR:
        pivot_position =
            robot_constants_->common()->pivot_set_points()->algae_processor();
        break;
      case PivotGoal::ALGAE_GROUND:
        pivot_position =
            robot_constants_->common()->pivot_set_points()->algae_ground();
        break;
      case PivotGoal::INTAKE_HP_BACKUP:
        pivot_position =
            robot_constants_->common()->pivot_set_points()->intake_hp_backup();
        break;
      case PivotGoal::CLIMB:
        pivot_position =
            robot_constants_->common()->pivot_set_points()->climb();
        break;
    }
    if (unsafe_goal->robot_side() == RobotSide::BACK) {
      pivot_position *= -1;
    }
  }

  PopulateStaticZeroingSingleDOFProfiledSubsystemGoal(pivot_goal_builder.get(),
                                                      pivot_position);

  const frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoal
      *pivot_goal = &pivot_goal_builder->AsFlatbuffer();

  const flatbuffers::Offset<PotAndAbsoluteEncoderProfiledJointStatus>
      pivot_status_offset = pivot_.Iterate(
          pivot_goal, position->pivot(),
          output != nullptr ? &(output_struct.pivot_voltage) : nullptr,
          status->fbb());

  double climber_current = 0.0;

  if (unsafe_goal != nullptr) {
    switch (unsafe_goal->climber_goal()) {
      case (ClimberGoal::NEUTRAL):
        break;
      case (ClimberGoal::CLIMB):
        climber_current =
            robot_constants_->common()->climber_current()->climb();
        break;
      case (ClimberGoal::RETRACT):
        climber_current =
            robot_constants_->common()->climber_current()->retract();
    }
  }
  output_struct.climber_current = climber_current;

  aos::fbs::FixedStackAllocator<aos::fbs::Builder<
      frc971::control_loops::
          StaticZeroingSingleDOFProfiledSubsystemGoalStatic>::kBufferSize>
      wrist_goal_buffer;

  aos::fbs::Builder<
      frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoalStatic>
      wrist_goal_builder(&wrist_goal_buffer);

  double wrist_position =
      robot_constants_->common()->wrist_set_points()->neutral();
  if (wrist_can_move_ && unsafe_goal != nullptr) {
    switch (unsafe_goal->wrist_goal()) {
      case (WristGoal::NEUTRAL):
        break;
      case (WristGoal::INTAKE_HP):
        wrist_position =
            robot_constants_->common()->wrist_set_points()->intake_hp();
        break;
      case (WristGoal::INTAKE_GROUND):
        wrist_position =
            robot_constants_->common()->wrist_set_points()->intake_ground();
        break;
      case WristGoal::SCORE_L1:
        wrist_position =
            robot_constants_->common()->wrist_set_points()->score_l1();
        break;
      case WristGoal::SCORE_L2:
        wrist_position =
            robot_constants_->common()->wrist_set_points()->score_l2();
        break;
      case WristGoal::SCORE_L3:
        wrist_position =
            robot_constants_->common()->wrist_set_points()->score_l3();
        break;
      case WristGoal::SCORE_L4:
        wrist_position =
            robot_constants_->common()->wrist_set_points()->score_l4();
        break;
      case WristGoal::ALGAE_L2:
        wrist_position =
            robot_constants_->common()->wrist_set_points()->algae_l2();
        break;
      case WristGoal::ALGAE_L3:
        wrist_position =
            robot_constants_->common()->wrist_set_points()->algae_l3();
        break;
      case WristGoal::BARGE:
        wrist_position =
            robot_constants_->common()->wrist_set_points()->barge();
        break;
      case WristGoal::ALGAE_PROCESSOR:
        wrist_position =
            robot_constants_->common()->wrist_set_points()->algae_processor();
        break;
      case WristGoal::ALGAE_GROUND:
        wrist_position =
            robot_constants_->common()->wrist_set_points()->algae_ground();
        break;
      case WristGoal::INTAKE_HP_BACKUP:
        wrist_position =
            robot_constants_->common()->wrist_set_points()->intake_hp_backup();
        break;
      case WristGoal::CLIMB:
        wrist_position =
            robot_constants_->common()->wrist_set_points()->climb();
        break;
    }
  }

  if (unsafe_goal != nullptr && unsafe_goal->robot_side() == RobotSide::BACK) {
    wrist_position *= -1;
  }

  PopulateStaticZeroingSingleDOFProfiledSubsystemGoal(wrist_goal_builder.get(),
                                                      wrist_position);

  const frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoal
      *wrist_goal = &wrist_goal_builder->AsFlatbuffer();

  const flatbuffers::Offset<
      frc971::control_loops::AbsoluteEncoderProfiledJointStatus>
      wrist_status_offset = wrist_.Iterate(
          wrist_goal, position->wrist(),
          output != nullptr ? &(output_struct.wrist_voltage) : nullptr,
          status->fbb());

  if (output) {
    output->CheckOk(output->Send(Output::Pack(*output->fbb(), &output_struct)));
  }

  Status::Builder status_builder = status->MakeBuilder<Status>();

  const bool zeroed = elevator_.zeroed() && pivot_.zeroed() && wrist_.zeroed();
  const bool estopped =
      elevator_.estopped() || pivot_.estopped() || wrist_.estopped();

  status_builder.add_zeroed(zeroed);
  status_builder.add_estopped(estopped);
  status_builder.add_elevator(elevator_status_offset);
  status_builder.add_wrist(wrist_status_offset);
  status_builder.add_pivot(pivot_status_offset);
  status_builder.add_end_effector_state(end_effector_status);
  status_builder.add_intake_complete(intake_complete_);

  (void)status->Send(status_builder.Finish());
}
}  // namespace y2025::control_loops::superstructure
