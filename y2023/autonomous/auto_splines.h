#ifndef Y2023_AUTONOMOUS_AUTO_SPLINES_H_
#define Y2023_AUTONOMOUS_AUTO_SPLINES_H_

#include "aos/events/event_loop.h"
#include "aos/flatbuffer_merge.h"
#include "frc971/control_loops/control_loops_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_goal_generated.h"
#include "frc971/input/joystick_state_generated.h"
/*

  The cooridinate system for the autonomous splines is the same as the spline
  python generator and drivetrain spline systems.

*/

namespace y2023 {
namespace autonomous {

class AutonomousSplines {
 public:
  AutonomousSplines()
      : test_spline_(aos::JsonFileToFlatbuffer<frc971::MultiSpline>(
            "splines/test_spline.json")),
        spline_1_(aos::JsonFileToFlatbuffer<frc971::MultiSpline>(
            "splines/spline.0.json")),
        spline_2_(aos::JsonFileToFlatbuffer<frc971::MultiSpline>(
            "splines/spline.1.json")),
        spline_3_(aos::JsonFileToFlatbuffer<frc971::MultiSpline>(
            "splines/spline.2.json")),
        spline_4_(aos::JsonFileToFlatbuffer<frc971::MultiSpline>(
            "splines/spline.3.json")),
        splinecable_1_(aos::JsonFileToFlatbuffer<frc971::MultiSpline>(
            "splines/splinecable.0.json")),
        splinecable_2_(aos::JsonFileToFlatbuffer<frc971::MultiSpline>(
            "splines/splinecable.1.json")),
        splinecable_3_(aos::JsonFileToFlatbuffer<frc971::MultiSpline>(
            "splines/splinecable.2.json")),
        splinecable_4_(aos::JsonFileToFlatbuffer<frc971::MultiSpline>(
            "splines/splinecable.3.json")) {}
  static flatbuffers::Offset<frc971::MultiSpline> BasicSSpline(
      aos::Sender<frc971::control_loops::drivetrain::SplineGoal>::Builder
          *builder,
      aos::Alliance alliance);
  static flatbuffers::Offset<frc971::MultiSpline> StraightLine(
      aos::Sender<frc971::control_loops::drivetrain::SplineGoal>::Builder
          *builder,
      aos::Alliance alliance);

  flatbuffers::Offset<frc971::MultiSpline> TestSpline(
      aos::Sender<frc971::control_loops::drivetrain::SplineGoal>::Builder
          *builder,
      aos::Alliance alliance);
  flatbuffers::Offset<frc971::MultiSpline> Spline1(
      aos::Sender<frc971::control_loops::drivetrain::SplineGoal>::Builder
          *builder,
      aos::Alliance alliance);
  flatbuffers::Offset<frc971::MultiSpline> Spline2(
      aos::Sender<frc971::control_loops::drivetrain::SplineGoal>::Builder
          *builder,
      aos::Alliance alliance);
  flatbuffers::Offset<frc971::MultiSpline> Spline3(
      aos::Sender<frc971::control_loops::drivetrain::SplineGoal>::Builder
          *builder,
      aos::Alliance alliance);
  flatbuffers::Offset<frc971::MultiSpline> Spline4(
      aos::Sender<frc971::control_loops::drivetrain::SplineGoal>::Builder
          *builder,
      aos::Alliance alliance);
  flatbuffers::Offset<frc971::MultiSpline> SplineCable1(
      aos::Sender<frc971::control_loops::drivetrain::SplineGoal>::Builder
          *builder,
      aos::Alliance alliance);
  flatbuffers::Offset<frc971::MultiSpline> SplineCable2(
      aos::Sender<frc971::control_loops::drivetrain::SplineGoal>::Builder
          *builder,
      aos::Alliance alliance);
  flatbuffers::Offset<frc971::MultiSpline> SplineCable3(
      aos::Sender<frc971::control_loops::drivetrain::SplineGoal>::Builder
          *builder,
      aos::Alliance alliance);
  flatbuffers::Offset<frc971::MultiSpline> SplineCable4(
      aos::Sender<frc971::control_loops::drivetrain::SplineGoal>::Builder
          *builder,
      aos::Alliance alliance);

 private:
  aos::FlatbufferDetachedBuffer<frc971::MultiSpline> test_spline_;
  aos::FlatbufferDetachedBuffer<frc971::MultiSpline> spline_1_;
  aos::FlatbufferDetachedBuffer<frc971::MultiSpline> spline_2_;
  aos::FlatbufferDetachedBuffer<frc971::MultiSpline> spline_3_;
  aos::FlatbufferDetachedBuffer<frc971::MultiSpline> spline_4_;
  aos::FlatbufferDetachedBuffer<frc971::MultiSpline> splinecable_1_;
  aos::FlatbufferDetachedBuffer<frc971::MultiSpline> splinecable_2_;
  aos::FlatbufferDetachedBuffer<frc971::MultiSpline> splinecable_3_;
  aos::FlatbufferDetachedBuffer<frc971::MultiSpline> splinecable_4_;
};

}  // namespace autonomous
}  // namespace y2023

#endif  // Y2023_AUTONOMOUS_AUTO_SPLINES_H_
