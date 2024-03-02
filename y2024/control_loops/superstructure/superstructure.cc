#include "y2024/control_loops/superstructure/superstructure.h"

#include <chrono>

#include "aos/events/event_loop.h"
#include "aos/flatbuffer_merge.h"
#include "aos/network/team_number.h"
#include "aos/time/time.h"
#include "frc971/shooter_interpolation/interpolation.h"
#include "frc971/zeroing/wrap.h"

DEFINE_bool(ignore_distance, false,
            "If true, ignore distance when shooting and obey joystick_reader");

// The threshold used when decided if the extend is close enough to a goal to
// continue.
constexpr double kExtendThreshold = 0.01;

constexpr double kTurretLoadingThreshold = 0.01;
constexpr double kAltitudeLoadingThreshold = 0.01;

constexpr std::chrono::milliseconds kExtraIntakingTime =
    std::chrono::milliseconds(500);

namespace y2024::control_loops::superstructure {

using ::aos::monotonic_clock;

using frc971::control_loops::AbsoluteEncoderProfiledJointStatus;
using frc971::control_loops::PotAndAbsoluteEncoderProfiledJointStatus;
using frc971::control_loops::RelativeEncoderProfiledJointStatus;

Superstructure::Superstructure(::aos::EventLoop *event_loop,
                               const ::std::string &name)
    : frc971::controls::ControlLoop<Goal, Position, Status, Output>(event_loop,
                                                                    name),
      constants_fetcher_(event_loop),
      robot_constants_(CHECK_NOTNULL(&constants_fetcher_.constants())),
      drivetrain_status_fetcher_(
          event_loop->MakeFetcher<frc971::control_loops::drivetrain::Status>(
              "/drivetrain")),
      joystick_state_fetcher_(
          event_loop->MakeFetcher<aos::JoystickState>("/aos")),
      intake_pivot_(robot_constants_->common()->intake_pivot(),
                    robot_constants_->robot()->intake_constants()),
      climber_(
          robot_constants_->common()->climber(),
          robot_constants_->robot()->climber_constants()->zeroing_constants()),
      shooter_(event_loop, robot_constants_),
      extend_(
          robot_constants_->common()->extend(),
          robot_constants_->robot()->extend_constants()->zeroing_constants()) {
  event_loop->SetRuntimeRealtimePriority(30);
}

bool PositionNear(double position, double goal, double threshold) {
  return std::abs(position - goal) < threshold;
}

void Superstructure::RunIteration(const Goal *unsafe_goal,
                                  const Position *position,
                                  aos::Sender<Output>::Builder *output,
                                  aos::Sender<Status>::Builder *status) {
  const monotonic_clock::time_point timestamp =
      event_loop()->context().monotonic_event_time;

  if (WasReset()) {
    AOS_LOG(ERROR, "WPILib reset, restarting\n");
    intake_pivot_.Reset();
    climber_.Reset();
    shooter_.Reset();
    extend_.Reset();
  }

  OutputT output_struct;

  // Handle Climber Goal separately from main superstructure state machine
  double climber_position =
      robot_constants_->common()->climber_set_points()->retract();

  if (unsafe_goal != nullptr) {
    switch (unsafe_goal->climber_goal()) {
      case ClimberGoal::FULL_EXTEND:
        climber_position =
            robot_constants_->common()->climber_set_points()->full_extend();
        break;
      case ClimberGoal::RETRACT:
        climber_position =
            robot_constants_->common()->climber_set_points()->retract();
        break;
      case ClimberGoal::STOWED:
        climber_position =
            robot_constants_->common()->climber_set_points()->stowed();
    }
  }

  // If we started off preloaded, skip to the ready state.
  if (unsafe_goal != nullptr && unsafe_goal->shooter_goal() &&
      unsafe_goal->shooter_goal()->preloaded()) {
    if (state_ != SuperstructureState::READY &&
        state_ != SuperstructureState::FIRING) {
      state_ = SuperstructureState::READY;
      catapult_requested_ = true;
    }
  }

  // Handle the intake pivot goal separately from the main superstructure state
  IntakeRollerStatus intake_roller_state = IntakeRollerStatus::NONE;
  double intake_pivot_position =
      robot_constants_->common()->intake_pivot_set_points()->retracted();

  if (unsafe_goal != nullptr) {
    switch (unsafe_goal->intake_goal()) {
      case IntakeGoal::INTAKE:
        intake_pivot_position =
            robot_constants_->common()->intake_pivot_set_points()->extended();
        intake_end_time_ = timestamp;
        break;
      case IntakeGoal::SPIT:
        intake_pivot_position =
            robot_constants_->common()->intake_pivot_set_points()->retracted();
        break;
      case IntakeGoal::NONE:
        intake_pivot_position =
            robot_constants_->common()->intake_pivot_set_points()->retracted();
        break;
    }
  }

  ExtendRollerStatus extend_roller_status = ExtendRollerStatus::IDLE;
  ExtendStatus extend_goal = ExtendStatus::RETRACTED;

  // True if the extend is moving towards a goal
  bool extend_moving = false;

  TransferRollerStatus transfer_roller_status = TransferRollerStatus::NONE;

  const ExtendSetPoints *extend_set_points =
      robot_constants_->common()->extend_set_points();

  // Checks if the extend is close enough to the retracted position to be
  // considered ready to accept note from the transfer rollers.
  const bool extend_at_retracted = PositionNear(
      extend_.position(), extend_set_points->retracted(), kExtendThreshold);

  // If true, the turret should be moved to the position to avoid collision with
  // the extend.
  bool move_turret_to_standby = false;

  // Superstructure state machine:
  // 1. IDLE. The intake is retracted and there is no note in the robot.
  // Wait for a intake goal to switch state to INTAKING if the extend is ready
  // 2. INTAKING. Intake the note and transfer it towards the extend.
  // Give intake, transfer, and extend rollers positive voltage to intake and
  // transfer. Switch to LOADED when the extend beambreak is triggered.
  // 3. LOADED. The note is in the extend and the extend is retracted.
  // Wait for a note goal to switch state to MOVING.
  // For AMP/TRAP goals, check that the turret is in a position to avoid
  // collision.
  // 4. MOVING. The extend is moving towards a goal (AMP, TRAP, or CATAPULT).
  // For CATAPULT goals, wait for the turret and altitude to be in a position to
  // accept the note from the extend.
  // Wait for the extend to reach the goal and switch state to READY if
  // AMP or TRAP, or to LOADING_CATAPULT if CATAPULT.
  // 5. LOADING_CATAPULT. The extend is at the position to load the catapult.
  // Activate the extend roller to transfer the note to the catapult.
  // Switch state to READY when the catapult beambreak is triggered.
  // 6. READY. Ready for fire command. The note is either loaded in the catapult
  // or in the extend and at the AMP or TRAP position. Wait for a fire command.
  // 7. FIRING. The note is being fired, either from the extend or the catapult.
  // Switch state back to IDLE when the note is fired.

  std::optional<bool> turret_ready_for_extend_move;
  switch (state_) {
    case SuperstructureState::IDLE:
      if (unsafe_goal != nullptr &&
          unsafe_goal->intake_goal() == IntakeGoal::INTAKE &&
          extend_at_retracted) {
        state_ = SuperstructureState::INTAKING;
      }

      extend_goal = ExtendStatus::RETRACTED;
      catapult_requested_ = false;
      break;
    case SuperstructureState::INTAKING:
      // Switch to LOADED state when the extend beambreak is triggered
      // meaning the note is loaded in the extend
      if (position->extend_beambreak()) {
        state_ = SuperstructureState::LOADED;
      }
      intake_roller_state = IntakeRollerStatus::INTAKING;
      transfer_roller_status = TransferRollerStatus::TRANSFERING_IN;
      extend_roller_status = ExtendRollerStatus::TRANSFERING_TO_EXTEND;
      extend_goal = ExtendStatus::RETRACTED;

      if (!catapult_requested_ && unsafe_goal != nullptr &&
          unsafe_goal->note_goal() == NoteGoal::CATAPULT) {
        catapult_requested_ = true;
      }

      // If we are no longer requesting INTAKE or we are no longer requesting
      // an INTAKE goal, wait 0.5 seconds then go back to IDLE.
      if (!(unsafe_goal != nullptr &&
            unsafe_goal->intake_goal() == IntakeGoal::INTAKE) &&
          timestamp > intake_end_time_ + kExtraIntakingTime) {
        state_ = SuperstructureState::IDLE;
      }

      break;
    case SuperstructureState::LOADED:
      if (!position->extend_beambreak() && !position->catapult_beambreak()) {
        state_ = SuperstructureState::IDLE;
      }

      if (catapult_requested_ == true) {
        state_ = SuperstructureState::MOVING;
        break;
      }

      if (unsafe_goal != nullptr &&
          unsafe_goal->note_goal() != NoteGoal::NONE) {
        // If the goal is AMP or TRAP, check if the turret is in a position to
        // avoid collision when the extend moves.
        if (unsafe_goal->note_goal() == NoteGoal::AMP ||
            unsafe_goal->note_goal() == NoteGoal::TRAP) {
          turret_ready_for_extend_move =
              PositionNear(shooter_.turret().estimated_position(),
                           robot_constants_->common()
                               ->turret_avoid_extend_collision_position(),
                           kTurretLoadingThreshold);
          transfer_roller_status = TransferRollerStatus::TRANSFERING_IN;

          if (turret_ready_for_extend_move.value()) {
            state_ = SuperstructureState::MOVING;
          } else {
            move_turret_to_standby = true;
          }
        } else if (unsafe_goal->note_goal() == NoteGoal::CATAPULT) {
          // If catapult is requested, switch to MOVING state
          state_ = SuperstructureState::MOVING;
        }
      }
      extend_goal = ExtendStatus::RETRACTED;
      if (!catapult_requested_ && unsafe_goal != nullptr &&
          unsafe_goal->note_goal() == NoteGoal::CATAPULT) {
        catapult_requested_ = true;
      }
      break;
    case SuperstructureState::MOVING:
      transfer_roller_status = TransferRollerStatus::TRANSFERING_IN;

      if (catapult_requested_) {
        extend_goal = ExtendStatus::CATAPULT;

        // Check if the extend is at the position to load the catapult
        bool extend_ready_for_catapult_transfer =
            PositionNear(extend_.position(), extend_set_points->catapult(),
                         kExtendThreshold);

        // Check if the turret is at the position to accept the note from extend
        bool turret_ready_for_load =
            PositionNear(shooter_.turret().estimated_position(),
                         robot_constants_->common()->turret_loading_position(),
                         kTurretLoadingThreshold);

        // Check if the altitude is at the position to accept the note from
        // extend
        bool altitude_ready_for_load = PositionNear(
            shooter_.altitude().estimated_position(),
            robot_constants_->common()->altitude_loading_position(),
            kAltitudeLoadingThreshold);

        if (extend_ready_for_catapult_transfer && turret_ready_for_load &&
            altitude_ready_for_load) {
          state_ = SuperstructureState::LOADING_CATAPULT;
        }
      } else {
        if (unsafe_goal != nullptr) {
          switch (unsafe_goal->note_goal()) {
            case NoteGoal::AMP:
              extend_goal = ExtendStatus::AMP;
              move_turret_to_standby = true;
              // Check if the extend is at the AMP position and if it is
              // switch to READY state
              if (PositionNear(extend_.position(), extend_set_points->amp(),
                               kExtendThreshold)) {
                state_ = SuperstructureState::READY;
              }
              break;
            case NoteGoal::TRAP:
              extend_goal = ExtendStatus::TRAP;
              move_turret_to_standby = true;
              // Check if the extend is at the TRAP position and if it is
              // switch to READY state
              if (PositionNear(extend_.position(), extend_set_points->trap(),
                               kExtendThreshold)) {
                state_ = SuperstructureState::READY;
              }
              break;
            case NoteGoal::NONE:
              extend_goal = ExtendStatus::RETRACTED;
              move_turret_to_standby = true;
              if (extend_at_retracted) {
                state_ = SuperstructureState::LOADED;
              }
              break;
            case NoteGoal::CATAPULT:
              catapult_requested_ = true;
              extend_goal = ExtendStatus::CATAPULT;
              break;
          }
        }
      }

      extend_moving = true;
      break;
    case SuperstructureState::LOADING_CATAPULT:
      extend_moving = false;
      extend_goal = ExtendStatus::CATAPULT;
      extend_roller_status = ExtendRollerStatus::TRANSFERING_TO_CATAPULT;

      // Switch to READY state when the catapult beambreak is triggered
      if (position->catapult_beambreak()) {
        state_ = SuperstructureState::READY;
      }
      break;
    case SuperstructureState::READY:
      extend_moving = false;

      // Switch to FIRING state when the fire button is pressed
      if (unsafe_goal != nullptr && unsafe_goal->fire()) {
        state_ = SuperstructureState::FIRING;
      }

      if (catapult_requested_) {
        extend_goal = ExtendStatus::CATAPULT;
      } else {
        if (unsafe_goal != nullptr) {
          if (unsafe_goal->note_goal() == NoteGoal::AMP) {
            extend_goal = ExtendStatus::AMP;
            move_turret_to_standby = true;
          } else if (unsafe_goal->note_goal() == NoteGoal::TRAP) {
            extend_goal = ExtendStatus::TRAP;
            move_turret_to_standby = true;
          } else {
            extend_goal = ExtendStatus::RETRACTED;
            extend_moving = true;
            state_ = SuperstructureState::MOVING;
          }
        }
      }

      break;
    case SuperstructureState::FIRING:
      if (catapult_requested_) {
        extend_goal = ExtendStatus::CATAPULT;

        // Reset the state to IDLE when the game piece is fired from the
        // catapult. We consider the game piece to be fired from the catapult
        // when the catapultbeambreak is no longer triggered.
        if (!position->catapult_beambreak()) {
          state_ = SuperstructureState::IDLE;
        }
      } else {
        move_turret_to_standby = true;
        if (unsafe_goal != nullptr &&
            unsafe_goal->note_goal() == NoteGoal::AMP) {
          extend_roller_status = ExtendRollerStatus::SCORING_IN_AMP;
          extend_goal = ExtendStatus::AMP;
        } else if (unsafe_goal != nullptr &&
                   unsafe_goal->note_goal() == NoteGoal::TRAP) {
          extend_roller_status = ExtendRollerStatus::SCORING_IN_TRAP;
          extend_goal = ExtendStatus::TRAP;
        }

        // Reset the state to IDLE when the game piece is fired from the extend.
        // We consider the game piece to be fired from the extend when
        // the extend beambreak is no longer triggered and the fire button is
        // released.
        if (!position->extend_beambreak() && unsafe_goal != nullptr &&
            !unsafe_goal->fire()) {
          state_ = SuperstructureState::IDLE;
        }
      }
      break;
  }

  if (unsafe_goal != nullptr &&
      unsafe_goal->intake_goal() == IntakeGoal::SPIT) {
    intake_roller_state = IntakeRollerStatus::SPITTING;
    transfer_roller_status = TransferRollerStatus::TRANSFERING_OUT;
  }

  // Update Intake Roller voltage based on status from state machine.
  switch (intake_roller_state) {
    case IntakeRollerStatus::NONE:
      output_struct.intake_roller_voltage = 0.0;
      break;
    case IntakeRollerStatus::SPITTING:
      output_struct.intake_roller_voltage =
          robot_constants_->common()->intake_roller_voltages()->spitting();
      break;
    case IntakeRollerStatus::INTAKING:
      output_struct.intake_roller_voltage =
          robot_constants_->common()->intake_roller_voltages()->intaking();
      break;
  }

  // Update Transfer Roller voltage based on status from state machine.
  switch (transfer_roller_status) {
    case TransferRollerStatus::NONE:
      output_struct.transfer_roller_voltage = 0.0;
      break;
    case TransferRollerStatus::TRANSFERING_IN:
      output_struct.transfer_roller_voltage =
          robot_constants_->common()->transfer_roller_voltages()->transfer_in();
      break;
    case TransferRollerStatus::TRANSFERING_OUT:
      output_struct.transfer_roller_voltage = robot_constants_->common()
                                                  ->transfer_roller_voltages()
                                                  ->transfer_out();
      break;
  }

  // Update Extend Roller voltage based on status from state machine.
  const ExtendRollerVoltages *extend_roller_voltages =
      robot_constants_->common()->extend_roller_voltages();
  switch (extend_roller_status) {
    case ExtendRollerStatus::IDLE:
      // No voltage applied when idle
      output_struct.extend_roller_voltage = 0.0;
      break;
    case ExtendRollerStatus::TRANSFERING_TO_EXTEND:
      output_struct.extend_roller_voltage = extend_roller_voltages->scoring();
      break;
    case ExtendRollerStatus::SCORING_IN_AMP:
      [[fallthrough]];
    case ExtendRollerStatus::SCORING_IN_TRAP:
      // Apply scoring voltage during scoring in amp or trap
      output_struct.extend_roller_voltage = extend_roller_voltages->scoring();
      break;
    case ExtendRollerStatus::TRANSFERING_TO_CATAPULT:
      // Apply scoring voltage during transferring to catapult
      output_struct.extend_roller_voltage = extend_roller_voltages->scoring();
      break;
  }

  double extend_position = 0.0;

  if (unsafe_goal != nullptr && unsafe_goal->note_goal() == NoteGoal::TRAP) {
    extend_goal = ExtendStatus::TRAP;
    move_turret_to_standby = true;
  }

  // In lieu of having full collision avoidance ready, move the turret out of
  // the way whenever the extend is raised too much.
  if (extend_.position() > 0.05) {
    move_turret_to_standby = true;
  }

  // Set the extend position based on the state machine output
  switch (extend_goal) {
    case ExtendStatus::RETRACTED:
      extend_position = extend_set_points->retracted();
      break;
    case ExtendStatus::AMP:
      extend_position = extend_set_points->amp();
      break;
    case ExtendStatus::TRAP:
      extend_position = extend_set_points->trap();
      break;
    case ExtendStatus::CATAPULT:
      extend_position = extend_set_points->catapult();
      break;
    case ExtendStatus::MOVING:
      // Should never happen
      break;
  }

  // Set the extend status based on the state machine output
  // If the extend is moving, the status is MOVING, otherwise it is the same
  // as extend_status
  ExtendStatus extend_status =
      (extend_moving ? ExtendStatus::MOVING : extend_goal);

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

  // TODO(max): Change how we handle the collision with the turret and
  // intake to be clearer
  const flatbuffers::Offset<ShooterStatus> shooter_status_offset =
      shooter_.Iterate(
          position,
          unsafe_goal != nullptr ? unsafe_goal->shooter_goal() : nullptr,
          unsafe_goal != nullptr ? unsafe_goal->fire() : false,
          output != nullptr ? &output_struct.catapult_voltage : nullptr,
          output != nullptr ? &output_struct.altitude_voltage : nullptr,
          output != nullptr ? &output_struct.turret_voltage : nullptr,
          output != nullptr ? &output_struct.retention_roller_voltage : nullptr,
          output != nullptr
              ? &output_struct.retention_roller_stator_current_limit
              : nullptr,
          robot_state().voltage_battery(), &collision_avoidance_,
          intake_pivot_.estimated_position(), &max_intake_pivot_position,
          &min_intake_pivot_position, move_turret_to_standby, status->fbb(),
          timestamp);

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

  aos::FlatbufferFixedAllocatorArray<
      frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoal, 512>
      note_goal_buffer;

  note_goal_buffer.Finish(
      frc971::control_loops::CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
          *note_goal_buffer.fbb(), extend_position));

  const frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoal
      *note_goal = &note_goal_buffer.message();

  const flatbuffers::Offset<PotAndAbsoluteEncoderProfiledJointStatus>
      extend_status_offset = extend_.Iterate(
          note_goal, position->extend(),
          output != nullptr ? &output_struct.extend_voltage : nullptr,
          status->fbb());

  if (output) {
    output->CheckOk(output->Send(Output::Pack(*output->fbb(), &output_struct)));
  }

  Status::Builder status_builder = status->MakeBuilder<Status>();

  const bool zeroed = intake_pivot_.zeroed() && climber_.zeroed() &&
                      shooter_.zeroed() && extend_.zeroed();
  const bool estopped = intake_pivot_.estopped() || climber_.estopped() ||
                        shooter_.estopped() || extend_.estopped();

  status_builder.add_zeroed(zeroed);
  status_builder.add_estopped(estopped);
  status_builder.add_intake_roller(intake_roller_state);
  status_builder.add_intake_pivot(intake_pivot_status_offset);
  status_builder.add_transfer_roller(transfer_roller_status);
  status_builder.add_climber(climber_status_offset);
  status_builder.add_shooter(shooter_status_offset);
  status_builder.add_collided(collided);
  status_builder.add_extend_roller(extend_roller_status);
  status_builder.add_extend_status(extend_status);
  status_builder.add_extend(extend_status_offset);
  status_builder.add_state(state_);
  status_builder.add_extend_ready_for_transfer(extend_at_retracted);
  if (turret_ready_for_extend_move) {
    status_builder.add_turret_ready_for_extend_move(
        turret_ready_for_extend_move.value());
  }

  (void)status->Send(status_builder.Finish());
}

double Superstructure::robot_velocity() const {
  return (drivetrain_status_fetcher_.get() != nullptr
              ? drivetrain_status_fetcher_->robot_speed()
              : 0.0);
}

}  // namespace y2024::control_loops::superstructure
