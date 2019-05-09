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
  // Splines for 2 Panels on the far side of the Rocket

  // Path off of level 2 to the far side of the rocket with a panel
  static ::frc971::MultiSpline HABToFarRocket(bool is_left);

  // Path from the far side of the rocket to the loading station to pickup
  static ::frc971::MultiSpline FarRocketToHP(bool is_left);

  // Path from the far side of the rocket to the loading station to pickup
  static ::frc971::MultiSpline HPToFarRocket(bool is_left);

  // Path from the far side of the rocket to close to the loading station
  static ::frc971::MultiSpline FarRocketToNearHP(bool is_left);

  // Splines for 2 Panels on the far reaches of the cargo ship

  // Path from level 2 to 2nd cargo ship bay with a hatch panel
  static ::frc971::MultiSpline HABToSecondCargoShipBay(bool is_left);

  // Path from 2nd cargo ship bay to loading station
  static ::frc971::MultiSpline SecondCargoShipBayToHP(bool is_left);

  // Path from loading station to 3rd cargo ship bay with a hatch panel
  static ::frc971::MultiSpline HPToThirdCargoShipBay(bool is_left);

  // Path from 3rd cargo ship bay to near the loading station
  static ::frc971::MultiSpline ThirdCargoShipBayToNearHP(bool is_left);

  // Testing Splines
  static ::frc971::MultiSpline HPToNearRocketTest();
  static ::frc971::MultiSpline HabToFarRocketTest(bool is_left);
  static ::frc971::MultiSpline FarRocketToHPTest();

  static ::frc971::MultiSpline BasicSSpline();
  static ::frc971::MultiSpline StraightLine();
};

}  // namespace actors
}  // namespace y2019

#endif  // Y2019_ACTORS_AUTO_SPLINES_H_
