#include "y2024_swerve/swerve_publisher_lib.h"

y2024_swerve::SwervePublisher::SwervePublisher(aos::EventLoop *event_loop,
                                               aos::ExitHandle *exit_handle,
                                               const std::string &filename,
                                               double duration)
    : drivetrain_goal_sender_(
          event_loop->MakeSender<frc971::control_loops::swerve::GoalStatic>(
              "/drivetrain")) {
  event_loop
      ->AddTimer([this, filename]() {
        auto goal_builder = drivetrain_goal_sender_.MakeStaticBuilder();

        auto drivetrain_goal =
            aos::JsonFileToFlatbuffer<frc971::control_loops::swerve::Goal>(
                filename);
        CHECK(drivetrain_goal.Verify());
        CHECK(goal_builder->FromFlatbuffer(&drivetrain_goal.message()));

        goal_builder.CheckOk(goal_builder.Send());
      })
      ->Schedule(event_loop->monotonic_now(),
                 std::chrono::duration_cast<aos::monotonic_clock::duration>(
                     std::chrono::milliseconds(5)));
  event_loop->AddTimer([exit_handle]() { exit_handle->Exit(); })
      ->Schedule(event_loop->monotonic_now() +
                 std::chrono::milliseconds((int)duration));
}
y2024_swerve::SwervePublisher::~SwervePublisher() {
  auto builder = drivetrain_goal_sender_.MakeStaticBuilder();

  for (auto module_goal :
       {builder->add_front_left_goal(), builder->add_front_right_goal(),
        builder->add_back_left_goal(), builder->add_back_right_goal()}) {
    module_goal->set_rotation_angle(0.0);
    module_goal->set_translation_control_type_goal(
        frc971::control_loops::swerve::TranslationControlTypeGoal::CURRENT);
    module_goal->set_translation_current(0.0);
  }
  builder.CheckOk(builder.Send());
}
