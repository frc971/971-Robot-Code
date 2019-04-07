#ifndef Y2019_ACTORS_AUTO_SPLINES_H_
#define Y2019_ACTORS_AUTO_SPLINES_H_

#include "frc971/control_loops/control_loops.q.h"
/*

  The cooridinate system for the autonomous splines is the same as the spline
  python generator and drivetrain spline systems.

*/

namespace y2019 {
namespace actors {

class AutonomousSplines {
 public:
  // A spline that does an 's' cause that's what he wanted.
  static ::frc971::MultiSpline BasicSSpline();

  // Straight
  static ::frc971::MultiSpline StraightLine();

  // HP to near side rocket
  static ::frc971::MultiSpline HPToNearRocket();

  static ::frc971::MultiSpline HabToFarRocket();
  static ::frc971::MultiSpline FarRockettoHP();
};

}  // namespace actors
}  // namespace y2019

#endif // Y2019_ACTORS_AUTO_SPLINES_H_
