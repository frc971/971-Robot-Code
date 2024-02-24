#include "y2024/control_loops/superstructure/superstructure.h"

#include "aos/events/event_loop.h"
#include "aos/flatbuffer_merge.h"
#include "aos/network/team_number.h"
#include "frc971/shooter_interpolation/interpolation.h"
#include "frc971/zeroing/wrap.h"

DEFINE_bool(ignore_distance, false,
            "If true, ignore distance when shooting and obay joystick_reader");

namespace y2024::control_loops::superstructure {

using ::aos::monotonic_clock;

using frc971::control_loops::AbsoluteEncoderProfiledJointStatus;
using frc971::control_loops::PotAndAbsoluteEncoderProfiledJointStatus;
using frc971::control_loops::RelativeEncoderProfiledJointStatus;

Superstructure::Superstructure(::aos::EventLoop *event_loop,
                               std::shared_ptr<const constants::Values> values,
                               const ::std::string &name)
    : frc971::controls::ControlLoop<Goal, Position, Status, Output>(event_loop,
                                                                    name),
      values_(values),
      constants_fetcher_(event_loop),
      robot_constants_(CHECK_NOTNULL(&constants_fetcher_.constants())),
      drivetrain_status_fetcher_(
          event_loop->MakeFetcher<frc971::control_loops::drivetrain::Status>(
              "/drivetrain")),
      joystick_state_fetcher_(
          event_loop->MakeFetcher<aos::JoystickState>("/aos")),
      transfer_goal_(TransferRollerGoal::NONE),
      intake_pivot_(robot_constants_->common()->intake_pivot(),
                    robot_constants_->robot()->intake_constants()),
      climber_(
          robot_constants_->common()->climber(),
          robot_constants_->robot()->climber_constants()->zeroing_constants()),
      shooter_(event_loop, robot_constants_) {
  event_loop->SetRuntimeRealtimePriority(30);
}

void Superstructure::RunIteration(const Goal *unsafe_goal,
                                  const Position *position,
                                  aos::Sender<Output>::Builder *output,
                                  aos::Sender<Status>::Builder *status) {
  const monotonic_clock::time_point timestamp =
      event_loop()->context().monotonic_event_time;

  (void)timestamp;

  if (WasReset()) {
    AOS_LOG(ERROR, "WPILib reset, restarting\n");
    intake_pivot_.Reset();
    climber_.Reset();
    shooter_.Reset();
  }

  OutputT output_struct;

  double intake_pivot_position =
      robot_constants_->common()->intake_pivot_set_points()->retracted();

  if (unsafe_goal != nullptr &&
      unsafe_goal->intake_pivot_goal() == IntakePivotGoal::EXTENDED) {
    intake_pivot_position =
        robot_constants_->common()->intake_pivot_set_points()->extended();
  }

  IntakeRollerStatus intake_roller_state = IntakeRollerStatus::NONE;

  switch (unsafe_goal != nullptr ? unsafe_goal->intake_roller_goal()
                                 : IntakeRollerGoal::NONE) {
    case IntakeRollerGoal::NONE:
      output_struct.intake_roller_voltage = 0.0;
      break;
    case IntakeRollerGoal::SPIT:
      transfer_goal_ = TransferRollerGoal::TRANSFER_OUT;
      intake_roller_state = IntakeRollerStatus::SPITTING;
      output_struct.intake_roller_voltage =
          robot_constants_->common()->intake_roller_voltages()->spitting();
      break;
    case IntakeRollerGoal::INTAKE:
      transfer_goal_ = TransferRollerGoal::TRANSFER_IN;
      intake_roller_state = IntakeRollerStatus::INTAKING;
      output_struct.intake_roller_voltage =
          robot_constants_->common()->intake_roller_voltages()->intaking();
      break;
  }

  TransferRollerStatus transfer_roller_state = TransferRollerStatus::NONE;

  switch (unsafe_goal != nullptr ? transfer_goal_ : TransferRollerGoal::NONE) {
    case TransferRollerGoal::NONE:
      output_struct.transfer_roller_voltage = 0.0;
      break;
    case TransferRollerGoal::TRANSFER_IN:
      if (position->transfer_beambreak()) {
        transfer_goal_ = TransferRollerGoal::NONE;
        transfer_roller_state = TransferRollerStatus::NONE;
        output_struct.transfer_roller_voltage = 0.0;
        break;
      }
      transfer_roller_state = TransferRollerStatus::TRANSFERING_IN;
      output_struct.transfer_roller_voltage =
          robot_constants_->common()->transfer_roller_voltages()->transfer_in();
      break;
    case TransferRollerGoal::TRANSFER_OUT:
      transfer_roller_state = TransferRollerStatus::TRANSFERING_OUT;
      output_struct.transfer_roller_voltage = robot_constants_->common()
                                                  ->transfer_roller_voltages()
                                                  ->transfer_out();
      break;
  }

  double climber_position =
      robot_constants_->common()->climber_set_points()->retract();

  if (unsafe_goal != nullptr) {
    switch (unsafe_goal->climber_goal()) {
      case ClimberGoal::FULL_EXTEND:
        climber_position =
            robot_constants_->common()->climber_set_points()->full_extend();
        break;
      case ClimberGoal::HALF_EXTEND:
        climber_position =
            robot_constants_->common()->climber_set_points()->half_extend();
        break;
      case ClimberGoal::RETRACT:
        climber_position =
            robot_constants_->common()->climber_set_points()->retract();
        break;
      default:
        break;
    }
  }

  if (joystick_state_fetcher_.Fetch() &&
      joystick_state_fetcher_->has_alliance()) {
    alliance_ = joystick_state_fetcher_->alliance();
  }

  drivetrain_status_fetcher_.Fetch();

  const bool collided = collision_avoidance_.IsCollided(
      {.intake_pivot_position = intake_pivot_.estimated_position(),
       .turret_position = shooter_.turret().estimated_position()});

  aos::FlatbufferFixedAllocatorArray<
      frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoal, 512>
      climber_goal_buffer;

  climber_goal_buffer.Finish(
      frc971::control_loops::CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
          *climber_goal_buffer.fbb(), climber_position));

  const frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoal
      *climber_goal = &climber_goal_buffer.message();

  const flatbuffers::Offset<PotAndAbsoluteEncoderProfiledJointStatus>
      climber_status_offset = climber_.Iterate(
          climber_goal, position->climber(),
          output != nullptr ? &output_struct.climber_voltage : nullptr,
          status->fbb());

  double max_intake_pivot_position = 0;
  double min_intake_pivot_position = 0;

  aos::FlatbufferFixedAllocatorArray<
      frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoal, 512>
      intake_pivot_goal_buffer;

  intake_pivot_goal_buffer.Finish(
      frc971::control_loops::CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
          *intake_pivot_goal_buffer.fbb(), intake_pivot_position));

  const frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoal
      *intake_pivot_goal = &intake_pivot_goal_buffer.message();

  double *intake_output =
      (output != nullptr ? &output_struct.intake_pivot_voltage : nullptr);

  const bool disabled = intake_pivot_.Correct(
      intake_pivot_goal, position->intake_pivot(), intake_output == nullptr);

  // TODO(max): Change how we handle the collision with the turret and intake to
  // be clearer
  const flatbuffers::Offset<ShooterStatus> shooter_status_offset =
      shooter_.Iterate(
          position,
          unsafe_goal != nullptr ? unsafe_goal->shooter_goal() : nullptr,
          output != nullptr ? &output_struct.catapult_voltage : nullptr,
          output != nullptr ? &output_struct.altitude_voltage : nullptr,
          output != nullptr ? &output_struct.turret_voltage : nullptr,
          output != nullptr ? &output_struct.retention_roller_voltage : nullptr,
          robot_state().voltage_battery(), &collision_avoidance_,
          intake_pivot_.estimated_position(), &max_intake_pivot_position,
          &min_intake_pivot_position, status->fbb());

  intake_pivot_.set_min_position(min_intake_pivot_position);
  intake_pivot_.set_max_position(max_intake_pivot_position);

  // Calculate the loops for a cycle.
  const double voltage = intake_pivot_.UpdateController(disabled);

  intake_pivot_.UpdateObserver(voltage);

  // Write out all the voltages.
  if (intake_output) {
    *intake_output = voltage;
  }

  const flatbuffers::Offset<AbsoluteEncoderProfiledJointStatus>
      intake_pivot_status_offset = intake_pivot_.MakeStatus(status->fbb());

  if (output) {
    output->CheckOk(output->Send(Output::Pack(*output->fbb(), &output_struct)));
  }

  Status::Builder status_builder = status->MakeBuilder<Status>();

  const bool zeroed =
      intake_pivot_.zeroed() && climber_.zeroed() && shooter_.zeroed();
  const bool estopped =
      intake_pivot_.estopped() || climber_.estopped() || shooter_.estopped();

  status_builder.add_zeroed(zeroed);
  status_builder.add_estopped(estopped);
  status_builder.add_intake_roller(intake_roller_state);
  status_builder.add_intake_pivot(intake_pivot_status_offset);
  status_builder.add_transfer_roller(transfer_roller_state);
  status_builder.add_climber(climber_status_offset);
  status_builder.add_shooter(shooter_status_offset);
  status_builder.add_collided(collided);

  (void)status->Send(status_builder.Finish());
}

double Superstructure::robot_velocity() const {
  return (drivetrain_status_fetcher_.get() != nullptr
              ? drivetrain_status_fetcher_->robot_speed()
              : 0.0);
}

}  // namespace y2024::control_loops::superstructure
