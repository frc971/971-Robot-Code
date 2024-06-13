#include "frc971/control_loops/swerve/swerve_control_loops.h"

namespace frc971::control_loops::swerve {

SwerveControlLoops::SwerveControlLoops(::aos::EventLoop *event_loop,
                                       const ::std::string &name)
    : frc971::controls::ControlLoop<Goal, Position, StatusStatic, OutputStatic>(
          event_loop, name) {}

void SwerveControlLoops::RunIteration(
    const Goal *goal, const Position *position,
    aos::Sender<OutputStatic>::StaticBuilder *output_builder,
    aos::Sender<StatusStatic>::StaticBuilder *status_builder) {
  (void)goal, (void)position;

  if (output_builder != nullptr && goal != nullptr) {
    OutputStatic *output = output_builder->get();

    auto front_left_output = output->add_front_left_output();
    front_left_output->set_rotation_current(0);
    front_left_output->set_translation_current(
        goal->front_left_goal()->translation_current());

    auto front_right_output = output->add_front_right_output();
    front_right_output->set_rotation_current(0);
    front_right_output->set_translation_current(
        goal->front_right_goal()->translation_current());

    auto back_left_output = output->add_back_left_output();
    back_left_output->set_rotation_current(0);
    back_left_output->set_translation_current(
        goal->back_left_goal()->translation_current());

    auto back_right_output = output->add_back_right_output();
    back_right_output->set_rotation_current(0);
    back_right_output->set_translation_current(
        goal->back_right_goal()->translation_current());

    // Ignore the return value of Send
    output_builder->CheckOk(output_builder->Send());
  }

  if (status_builder != nullptr) {
    StatusStatic *status = status_builder->get();

    auto front_left_status = status->add_front_left_status();
    front_left_status->set_goal_translation_speed(0);
    front_left_status->set_translation_speed(0);
    front_left_status->add_rotation();

    auto front_right_status = status->add_front_right_status();
    front_right_status->set_goal_translation_speed(0);
    front_right_status->set_translation_speed(0);
    front_right_status->add_rotation();

    auto back_left_status = status->add_back_left_status();
    back_left_status->set_goal_translation_speed(0);
    back_left_status->set_translation_speed(0);
    back_left_status->add_rotation();

    auto back_right_status = status->add_back_right_status();
    back_right_status->set_goal_translation_speed(0);
    back_right_status->set_translation_speed(0);
    back_right_status->add_rotation();

    // Ignore the return value of Send
    status_builder->CheckOk(status_builder->Send());
  }
}

}  // namespace frc971::control_loops::swerve
