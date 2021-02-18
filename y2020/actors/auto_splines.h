#ifndef y2020_ACTORS_AUTO_SPLINES_H_
#define y2020_ACTORS_AUTO_SPLINES_H_

#include "aos/events/event_loop.h"
#include "aos/flatbuffer_merge.h"
#include "aos/robot_state/joystick_state_generated.h"
#include "frc971/control_loops/control_loops_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_goal_generated.h"
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
            "splines/test_spline.json")) {}

  static flatbuffers::Offset<frc971::MultiSpline> BasicSSpline(
      aos::Sender<frc971::control_loops::drivetrain::Goal>::Builder *builder,
      aos::Alliance alliance);
  static flatbuffers::Offset<frc971::MultiSpline> StraightLine(
      aos::Sender<frc971::control_loops::drivetrain::Goal>::Builder *builder);

  flatbuffers::Offset<frc971::MultiSpline> TestSpline(
      aos::Sender<frc971::control_loops::drivetrain::Goal>::Builder *builder) {
    return aos::CopyFlatBuffer<frc971::MultiSpline>(test_spline_,
                                                    builder->fbb());
  }

 private:
  aos::FlatbufferDetachedBuffer<frc971::MultiSpline> test_spline_;
};

}  // namespace actors
}  // namespace y2020

#endif  // y2020_ACTORS_AUTO_SPLINES_H_
