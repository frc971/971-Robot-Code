#include "y2020/control_loops/superstructure/superstructure.h"

#include "aos/events/event_loop.h"

namespace y2020 {
namespace control_loops {
namespace superstructure {

using frc971::control_loops::AbsoluteEncoderProfiledJointStatus;
using frc971::control_loops::PotAndAbsoluteEncoderProfiledJointStatus;

Superstructure::Superstructure(::aos::EventLoop *event_loop,
                               const ::std::string &name)
    : aos::controls::ControlLoop<Goal, Position, Status, Output>(event_loop,
                                                                 name),
      hood_(constants::GetValues().hood),
      intake_joint_(constants::GetValues().intake),
      turret_(constants::GetValues().turret.subsystem_params),
      shooter_() {
  event_loop->SetRuntimeRealtimePriority(30);
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

  OutputT output_struct;

  flatbuffers::Offset<AbsoluteEncoderProfiledJointStatus> hood_status_offset =
      hood_.Iterate(unsafe_goal != nullptr ? unsafe_goal->hood() : nullptr,
                    position->hood(),
                    output != nullptr ? &(output_struct.hood_voltage) : nullptr,
                    status->fbb());

  flatbuffers::Offset<AbsoluteEncoderProfiledJointStatus> intake_status_offset =
      intake_joint_.Iterate(
          unsafe_goal != nullptr ? unsafe_goal->intake() : nullptr,
          position->intake_joint(),
          output != nullptr ? &(output_struct.intake_joint_voltage) : nullptr,
          status->fbb());

  flatbuffers::Offset<PotAndAbsoluteEncoderProfiledJointStatus>
      turret_status_offset = turret_.Iterate(
          unsafe_goal != nullptr ? unsafe_goal->turret() : nullptr,
          position->turret(),
          output != nullptr ? &(output_struct.turret_voltage) : nullptr,
          status->fbb());

  flatbuffers::Offset<ShooterStatus> shooter_status_offset =
      shooter_.RunIteration(
          unsafe_goal != nullptr ? unsafe_goal->shooter() : nullptr,
          position->shooter(), status->fbb(),
          output != nullptr ? &(output_struct) : nullptr, position_timestamp);

  climber_.Iterate(unsafe_goal, output != nullptr ? &(output_struct) : nullptr);

  const AbsoluteEncoderProfiledJointStatus *const hood_status =
      GetMutableTemporaryPointer(*status->fbb(), hood_status_offset);

  const PotAndAbsoluteEncoderProfiledJointStatus *const turret_status =
      GetMutableTemporaryPointer(*status->fbb(), turret_status_offset);

  if (output != nullptr) {
    // Friction is a pain and putting a really high burden on the integrator.
    const double turret_velocity_sign = turret_status->velocity() * kTurretFrictionGain;
    output_struct.turret_voltage +=
        std::clamp(turret_velocity_sign, -kTurretFrictionVoltageLimit,
                   kTurretFrictionVoltageLimit);

    // Friction is a pain and putting a really high burden on the integrator.
    const double hood_velocity_sign = hood_status->velocity() * kHoodFrictionGain;
    output_struct.hood_voltage +=
        std::clamp(hood_velocity_sign, -kHoodFrictionVoltageLimit,
                   kHoodFrictionVoltageLimit);

    // And dither the output.
    time_ += 0.00505;
    output_struct.hood_voltage += 1.3 * std::sin(time_ * 2.0 * M_PI * 30.0);
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

  status->Send(status_builder.Finish());

  if (output != nullptr) {
    if (unsafe_goal) {
        output_struct.washing_machine_spinner_voltage = 6.0;
      if (unsafe_goal->shooting()) {
        output_struct.feeder_voltage = 6.0;
      } else {
        output_struct.feeder_voltage = 0.0;
      }
      output_struct.intake_roller_voltage = unsafe_goal->roller_voltage();
    } else {
      output_struct.intake_roller_voltage = 0.0;
    }
    output->Send(Output::Pack(*output->fbb(), &output_struct));
  }
}

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2020
