#include "frc971/control_loops/swerve/swerve_control_loops.h"

namespace frc971::control_loops::swerve {

SwerveControlLoops::SwerveControlLoops(
    ::aos::EventLoop *event_loop,
    const frc971::control_loops::
        StaticZeroingSingleDOFProfiledSubsystemCommonParams *rotation_params,
    const SwerveZeroing *zeroing_params, const ::std::string &name)
    : frc971::controls::ControlLoop<Goal, Position, StatusStatic, OutputStatic>(
          event_loop, name),
      front_left_(rotation_params, zeroing_params->front_left()),
      front_right_(rotation_params, zeroing_params->front_right()),
      back_left_(rotation_params, zeroing_params->back_left()),
      back_right_(rotation_params, zeroing_params->back_right()) {}

void SwerveControlLoops::RunIteration(
    const Goal *goal, const Position *position,
    aos::Sender<OutputStatic>::StaticBuilder *output_builder,
    aos::Sender<StatusStatic>::StaticBuilder *status_builder) {
  if (WasReset()) {
    front_left_.Reset();
    front_right_.Reset();
    back_left_.Reset();
    back_right_.Reset();
  }
  aos::FlatbufferFixedAllocatorArray<
      frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoal, 512>
      front_left_goal_buffer, front_right_goal_buffer, back_left_goal_buffer,
      back_right_goal_buffer;

  front_left_goal_buffer.Finish(
      frc971::control_loops::CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
          *front_left_goal_buffer.fbb(),
          (goal != nullptr && goal->has_front_left_goal())
              ? goal->front_left_goal()->rotation_angle()
              : 0.0));
  front_right_goal_buffer.Finish(
      frc971::control_loops::CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
          *front_right_goal_buffer.fbb(),
          (goal != nullptr && goal->has_front_right_goal())
              ? goal->front_right_goal()->rotation_angle()
              : 0.0));
  back_left_goal_buffer.Finish(
      frc971::control_loops::CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
          *back_left_goal_buffer.fbb(),
          (goal != nullptr && goal->has_back_left_goal())
              ? goal->back_left_goal()->rotation_angle()
              : 0.0));
  back_right_goal_buffer.Finish(
      frc971::control_loops::CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
          *back_right_goal_buffer.fbb(),
          (goal != nullptr && goal->has_back_right_goal())
              ? goal->back_right_goal()->rotation_angle()
              : 0.0));
  const bool disabled = front_left_.Correct(
      &front_left_goal_buffer.message(),
      position->front_left()->rotation_position(), output_builder == nullptr);
  front_right_.Correct(&front_right_goal_buffer.message(),
                       position->front_right()->rotation_position(),
                       output_builder == nullptr);
  back_left_.Correct(&back_left_goal_buffer.message(),
                     position->back_left()->rotation_position(),
                     output_builder == nullptr);
  back_right_.Correct(&back_right_goal_buffer.message(),
                      position->back_right()->rotation_position(),
                      output_builder == nullptr);

  const double front_left_voltage = front_left_.UpdateController(disabled);
  front_left_.UpdateObserver(front_left_voltage);
  const double front_right_voltage = front_right_.UpdateController(disabled);
  front_right_.UpdateObserver(front_right_voltage);
  const double back_left_voltage = back_left_.UpdateController(disabled);
  back_left_.UpdateObserver(back_left_voltage);
  const double back_right_voltage = back_right_.UpdateController(disabled);
  back_right_.UpdateObserver(back_right_voltage);
  (void)goal, (void)position;
  aos::FlatbufferFixedAllocatorArray<
      frc971::control_loops::AbsoluteEncoderProfiledJointStatus, 512>
      front_left_status_buffer, front_right_status_buffer,
      back_left_status_buffer, back_right_status_buffer;
  front_left_status_buffer.Finish(
      front_left_.MakeStatus(front_left_status_buffer.fbb()));
  front_right_status_buffer.Finish(
      front_right_.MakeStatus(front_right_status_buffer.fbb()));
  back_left_status_buffer.Finish(
      back_left_.MakeStatus(back_left_status_buffer.fbb()));
  back_right_status_buffer.Finish(
      back_right_.MakeStatus(back_right_status_buffer.fbb()));

  if (output_builder != nullptr) {
    OutputStatic *output = output_builder->get();

    auto front_left_output = output->add_front_left_output();
    front_left_output->set_rotation_current(front_left_voltage);
    front_left_output->set_translation_current(
        goal ? goal->front_left_goal()->translation_current() : 0.0);

    auto front_right_output = output->add_front_right_output();
    front_right_output->set_rotation_current(front_right_voltage);
    front_right_output->set_translation_current(
        goal ? goal->front_right_goal()->translation_current() : 0.0);

    auto back_left_output = output->add_back_left_output();
    back_left_output->set_rotation_current(back_left_voltage);
    back_left_output->set_translation_current(
        goal ? goal->back_left_goal()->translation_current() : 0.0);

    auto back_right_output = output->add_back_right_output();
    back_right_output->set_rotation_current(back_right_voltage);
    back_right_output->set_translation_current(
        goal ? goal->back_right_goal()->translation_current() : 0.0);

    // Ignore the return value of Send
    output_builder->CheckOk(output_builder->Send());
  }

  if (status_builder != nullptr) {
    StatusStatic *status = status_builder->get();

    auto front_left_status = status->add_front_left_status();
    PopulateSwerveModuleRotation(front_left_status,
                                 &front_left_status_buffer.message());

    auto front_right_status = status->add_front_right_status();
    PopulateSwerveModuleRotation(front_right_status,
                                 &front_right_status_buffer.message());

    auto back_left_status = status->add_back_left_status();
    PopulateSwerveModuleRotation(back_left_status,
                                 &back_left_status_buffer.message());

    auto back_right_status = status->add_back_right_status();
    PopulateSwerveModuleRotation(back_right_status,
                                 &back_right_status_buffer.message());

    // Ignore the return value of Send
    status_builder->CheckOk(status_builder->Send());
  }
}

}  // namespace frc971::control_loops::swerve
