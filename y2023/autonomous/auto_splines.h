#ifndef Y2023_ACTORS_AUTO_SPLINES_H_
#define Y2023_ACTORS_AUTO_SPLINES_H_

#include "aos/events/event_loop.h"
#include "frc971/control_loops/control_loops_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_goal_generated.h"
/*

  The cooridinate system for the autonomous splines is the same as the spline
  python generator and drivetrain spline systems.

*/

namespace y2023 {
namespace actors {

class AutonomousSplines {
 public:
  static flatbuffers::Offset<frc971::MultiSpline> BasicSSpline(
      aos::Sender<frc971::control_loops::drivetrain::Goal>::Builder *builder);
  static flatbuffers::Offset<frc971::MultiSpline> StraightLine(
      aos::Sender<frc971::control_loops::drivetrain::Goal>::Builder *builder);
};

}  // namespace actors
}  // namespace y2023

#endif  // Y2023_ACTORS_AUTO_SPLINES_H_
