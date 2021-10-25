#ifndef y2020_ACTORS_AUTO_SPLINES_H_
#define y2020_ACTORS_AUTO_SPLINES_H_

#include "aos/events/event_loop.h"
#include "aos/flatbuffer_merge.h"
#include "frc971/control_loops/control_loops_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_goal_generated.h"
#include "frc971/input/joystick_state_generated.h"
/*

  The cooridinate system for the autonomous splines is the same as the spline
  python generator and drivetrain spline systems.

*/

namespace y2020 {
namespace actors {

class AutonomousSplines {
 public:
  AutonomousSplines()
      : test_spline_(aos::JsonFileToFlatbuffer<frc971::MultiSpline>(
            "splines/test_spline.json")),
        target_aligned_1_(aos::JsonFileToFlatbuffer<frc971::MultiSpline>(
            "splines/target_aligned_1.json")),
        target_aligned_2_(aos::JsonFileToFlatbuffer<frc971::MultiSpline>(
            "splines/target_aligned_2.json")),
        target_offset_1_(aos::JsonFileToFlatbuffer<frc971::MultiSpline>(
            "splines/target_offset_1.json")),
        target_offset_2_(aos::JsonFileToFlatbuffer<frc971::MultiSpline>(
            "splines/target_offset_2.json")) {}

  static flatbuffers::Offset<frc971::MultiSpline> BasicSSpline(
      aos::Sender<frc971::control_loops::drivetrain::SplineGoal>::Builder
          *builder,
      aos::Alliance alliance);
  static flatbuffers::Offset<frc971::MultiSpline> StraightLine(
      aos::Sender<frc971::control_loops::drivetrain::SplineGoal>::Builder
          *builder);

  flatbuffers::Offset<frc971::MultiSpline> TestSpline(
      aos::Sender<frc971::control_loops::drivetrain::SplineGoal>::Builder
          *builder) {
    return aos::CopyFlatBuffer<frc971::MultiSpline>(test_spline_,
                                                    builder->fbb());
  }

  flatbuffers::Offset<frc971::MultiSpline> TargetAligned1(
      aos::Sender<frc971::control_loops::drivetrain::SplineGoal>::Builder
          *builder,
      aos::Alliance alliance);
  flatbuffers::Offset<frc971::MultiSpline> TargetAligned2(
      aos::Sender<frc971::control_loops::drivetrain::SplineGoal>::Builder
          *builder,
      aos::Alliance alliance);
  flatbuffers::Offset<frc971::MultiSpline> TargetOffset1(
      aos::Sender<frc971::control_loops::drivetrain::SplineGoal>::Builder
          *builder) {
    return aos::CopyFlatBuffer<frc971::MultiSpline>(target_offset_1_,
                                                    builder->fbb());
  }
  flatbuffers::Offset<frc971::MultiSpline> TargetOffset2(
      aos::Sender<frc971::control_loops::drivetrain::SplineGoal>::Builder
          *builder) {
    return aos::CopyFlatBuffer<frc971::MultiSpline>(target_offset_2_,
                                                    builder->fbb());
  }

 private:
  aos::FlatbufferDetachedBuffer<frc971::MultiSpline> test_spline_;
  aos::FlatbufferDetachedBuffer<frc971::MultiSpline> target_aligned_1_;
  aos::FlatbufferDetachedBuffer<frc971::MultiSpline> target_aligned_2_;
  aos::FlatbufferDetachedBuffer<frc971::MultiSpline> target_offset_1_;
  aos::FlatbufferDetachedBuffer<frc971::MultiSpline> target_offset_2_;
};

}  // namespace actors
}  // namespace y2020

#endif  // y2020_ACTORS_AUTO_SPLINES_H_
