#include "y2020/control_loops/superstructure/superstructure.h"

#include "aos/events/event_loop.h"

namespace y2020 {
namespace control_loops {
namespace superstructure {

using frc971::control_loops::AbsoluteAndAbsoluteEncoderProfiledJointStatus;
using frc971::control_loops::AbsoluteEncoderProfiledJointStatus;
using frc971::control_loops::PotAndAbsoluteEncoderProfiledJointStatus;

Superstructure::Superstructure(::aos::EventLoop *event_loop,
                               const ::std::string &name)
    : aos::controls::ControlLoop<Goal, Position, Status, Output>(event_loop,
                                                                 name),
      hood_(constants::GetValues().hood),
      intake_joint_(constants::GetValues().intake),
      turret_(constants::GetValues().turret.subsystem_params),
      drivetrain_status_fetcher_(
          event_loop->MakeFetcher<frc971::control_loops::drivetrain::Status>(
              "/drivetrain")),
      joystick_state_fetcher_(
          event_loop->MakeFetcher<aos::JoystickState>("/aos")) {
  event_loop->SetRuntimeRealtimePriority(30);
}

double Superstructure::robot_speed() const {
  return (drivetrain_status_fetcher_.get() != nullptr
              ? drivetrain_status_fetcher_->robot_speed()
              : 0.0);
}

void Superstructure::RunIteration(const Goal *unsafe_goal,
                                  const Position *position,
                                  aos::Sender<Output>::Builder *output,
                                  aos::Sender<Status>::Builder *status) {
  if (WasReset()) {
    AOS_LOG(ERROR, "WPILib reset, restarting\n");
    hood_.Reset();
    intake_joint_.Reset();
    turret_.Reset();
  }

  const aos::monotonic_clock::time_point position_timestamp =
      event_loop()->context().monotonic_event_time;

  if (drivetrain_status_fetcher_.Fetch()) {
    aos::Alliance alliance = aos::Alliance::kInvalid;
    joystick_state_fetcher_.Fetch();
    if (joystick_state_fetcher_.get() != nullptr) {
      alliance = joystick_state_fetcher_->alliance();
    }
    const turret::Aimer::WrapMode mode =
        (unsafe_goal != nullptr && unsafe_goal->shooting())
            ? turret::Aimer::WrapMode::kAvoidWrapping
            : turret::Aimer::WrapMode::kAvoidEdges;
    aimer_.Update(drivetrain_status_fetcher_.get(), alliance, mode,
                  turret::Aimer::ShotMode::kShootOnTheFly);
  }

  const float velocity = robot_speed();

  const flatbuffers::Offset<AimerStatus> aimer_status_offset =
      aimer_.PopulateStatus(status->fbb());

  const double distance_to_goal = aimer_.DistanceToGoal();

  aos::FlatbufferFixedAllocatorArray<
      frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoal, 64>
      hood_goal;
  aos::FlatbufferFixedAllocatorArray<ShooterGoal, 64> shooter_goal;

  constants::Values::ShotParams shot_params;
  if (constants::GetValues().shot_interpolation_table.GetInRange(
          distance_to_goal, &shot_params)) {
    hood_goal.Finish(frc971::control_loops::
                         CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
                             *hood_goal.fbb(), shot_params.hood_angle));

    shooter_goal.Finish(CreateShooterGoal(*shooter_goal.fbb(),
                                          shot_params.accelerator_power,
                                          shot_params.finisher_power));
  } else {
    hood_goal.Finish(
        frc971::control_loops::
            CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
                *hood_goal.fbb(), constants::GetValues().hood.range.upper));

    shooter_goal.Finish(CreateShooterGoal(*shooter_goal.fbb(), 0.0, 0.0));
  }

  OutputT output_struct;

  flatbuffers::Offset<AbsoluteAndAbsoluteEncoderProfiledJointStatus>
      hood_status_offset = hood_.Iterate(
          unsafe_goal != nullptr
              ? (unsafe_goal->hood_tracking() ? &hood_goal.message()
                                              : unsafe_goal->hood())
              : nullptr,
          position->hood(),
          output != nullptr ? &(output_struct.hood_voltage) : nullptr,
          status->fbb());

  if (unsafe_goal != nullptr) {
    if (unsafe_goal->shooting() &&
        shooting_start_time_ == aos::monotonic_clock::min_time) {
      shooting_start_time_ = position_timestamp;
    }

    if (unsafe_goal->shooting()) {
      constexpr std::chrono::milliseconds kPeriod =
          std::chrono::milliseconds(250);
      if ((position_timestamp - shooting_start_time_) % (kPeriod * 2) <
          kPeriod) {
        intake_joint_.set_min_position(-0.25);
      } else {
        intake_joint_.set_min_position(-0.75);
      }
    } else {
      intake_joint_.clear_min_position();
    }

    if (!unsafe_goal->shooting()) {
      shooting_start_time_ = aos::monotonic_clock::min_time;
    }
  }

  flatbuffers::Offset<AbsoluteEncoderProfiledJointStatus> intake_status_offset =
      intake_joint_.Iterate(
          unsafe_goal != nullptr ? unsafe_goal->intake() : nullptr,
          position->intake_joint(),
          output != nullptr ? &(output_struct.intake_joint_voltage) : nullptr,
          status->fbb());

  const frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoal
      *turret_goal = unsafe_goal != nullptr ? (unsafe_goal->turret_tracking()
                                                   ? aimer_.TurretGoal()
                                                   : unsafe_goal->turret())
                                            : nullptr;

  flatbuffers::Offset<PotAndAbsoluteEncoderProfiledJointStatus>
      turret_status_offset = turret_.Iterate(
          turret_goal, position->turret(),
          output != nullptr ? &(output_struct.turret_voltage) : nullptr,
          status->fbb());

  flatbuffers::Offset<ShooterStatus> shooter_status_offset =
      shooter_.RunIteration(
          unsafe_goal != nullptr
              ? (unsafe_goal->shooter_tracking() ? &shooter_goal.message()
                                                 : unsafe_goal->shooter())
              : nullptr,
          position->shooter(), status->fbb(),
          output != nullptr ? &(output_struct) : nullptr, position_timestamp);

  climber_.Iterate(unsafe_goal, output != nullptr ? &(output_struct) : nullptr);

  const AbsoluteAndAbsoluteEncoderProfiledJointStatus *const hood_status =
      GetMutableTemporaryPointer(*status->fbb(), hood_status_offset);

  const PotAndAbsoluteEncoderProfiledJointStatus *const turret_status =
      GetMutableTemporaryPointer(*status->fbb(), turret_status_offset);

  if (output != nullptr) {
    // Friction is a pain and putting a really high burden on the integrator.
    const double turret_velocity_sign =
        turret_status->velocity() * kTurretFrictionGain;
    output_struct.turret_voltage +=
        std::clamp(turret_velocity_sign, -kTurretFrictionVoltageLimit,
                   kTurretFrictionVoltageLimit);
    output_struct.turret_voltage =
        std::clamp(output_struct.turret_voltage, -turret_.operating_voltage(),
                   turret_.operating_voltage());
  }

  bool zeroed;
  bool estopped;

  {
    const AbsoluteEncoderProfiledJointStatus *const intake_status =
        GetMutableTemporaryPointer(*status->fbb(), intake_status_offset);

    zeroed = hood_status->zeroed() && intake_status->zeroed() &&
             turret_status->zeroed();
    estopped = hood_status->estopped() || intake_status->estopped() ||
               turret_status->estopped();
  }

  Status::Builder status_builder = status->MakeBuilder<Status>();

  status_builder.add_zeroed(zeroed);
  status_builder.add_estopped(estopped);

  status_builder.add_hood(hood_status_offset);
  status_builder.add_intake(intake_status_offset);
  status_builder.add_turret(turret_status_offset);
  status_builder.add_shooter(shooter_status_offset);
  status_builder.add_aimer(aimer_status_offset);

  status->Send(status_builder.Finish());

  if (output != nullptr) {
    if (unsafe_goal) {
      output_struct.washing_machine_spinner_voltage = 0.0;
      if (unsafe_goal->shooting()) {
        if (shooter_.ready() && shooter_.finisher_goal() > 10.0 &&
            shooter_.accelerator_goal() > 10.0) {
          output_struct.feeder_voltage = 12.0;
        } else {
          output_struct.feeder_voltage = 0.0;
        }
        output_struct.washing_machine_spinner_voltage = 5.0;
        output_struct.intake_roller_voltage = 3.0;
      } else {
        output_struct.feeder_voltage = 0.0;
        output_struct.intake_roller_voltage =
            unsafe_goal->roller_voltage() +
            std::max(velocity * unsafe_goal->roller_speed_compensation(), 0.0f);
      }
    } else {
      output_struct.intake_roller_voltage = 0.0;
    }
    output->Send(Output::Pack(*output->fbb(), &output_struct));
  }
}

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2020
