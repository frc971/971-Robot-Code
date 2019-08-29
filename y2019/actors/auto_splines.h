#ifndef Y2019_ACTORS_AUTO_SPLINES_H_
#define Y2019_ACTORS_AUTO_SPLINES_H_

#include "aos/events/event_loop.h"
#include "frc971/control_loops/control_loops_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_goal_generated.h"
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
  static flatbuffers::Offset<frc971::MultiSpline> HABToFarRocket(
      aos::Sender<frc971::control_loops::drivetrain::Goal>::Builder *builder,
      bool is_left);

  // Path from the far side of the rocket to the loading station to pickup
  static flatbuffers::Offset<frc971::MultiSpline> FarRocketToHP(
      aos::Sender<frc971::control_loops::drivetrain::Goal>::Builder *builder,
      bool is_left);

  // Path from the far side of the rocket to the loading station to pickup
  static flatbuffers::Offset<frc971::MultiSpline> HPToFarRocket(
      aos::Sender<frc971::control_loops::drivetrain::Goal>::Builder *builder,
      bool is_left);

  // Path from the far side of the rocket to close to the loading station
  static flatbuffers::Offset<frc971::MultiSpline> FarRocketToNearHP(
      aos::Sender<frc971::control_loops::drivetrain::Goal>::Builder *builder,
      bool is_left);

  // Splines for 2 Panels on the far reaches of the cargo ship

  // Path from level 2 to 2nd cargo ship bay with a hatch panel
  static flatbuffers::Offset<frc971::MultiSpline> HABToSecondCargoShipBay(
      aos::Sender<frc971::control_loops::drivetrain::Goal>::Builder *builder,
      bool is_left);

  // Path from 2nd cargo ship bay to loading station
  static flatbuffers::Offset<frc971::MultiSpline> SecondCargoShipBayToHP(
      aos::Sender<frc971::control_loops::drivetrain::Goal>::Builder *builder,
      bool is_left);

  // Path from loading station to 3rd cargo ship bay with a hatch panel
  static flatbuffers::Offset<frc971::MultiSpline> HPToThirdCargoShipBay(
      aos::Sender<frc971::control_loops::drivetrain::Goal>::Builder *builder,
      bool is_left);

  // Path from 3rd cargo ship bay to near the loading station
  static flatbuffers::Offset<frc971::MultiSpline> ThirdCargoShipBayToNearHP(
      aos::Sender<frc971::control_loops::drivetrain::Goal>::Builder *builder,
      bool is_left);

  // Testing Splines
  static flatbuffers::Offset<frc971::MultiSpline> HPToNearRocketTest(
      aos::Sender<frc971::control_loops::drivetrain::Goal>::Builder *builder);
  static flatbuffers::Offset<frc971::MultiSpline> HabToFarRocketTest(
      aos::Sender<frc971::control_loops::drivetrain::Goal>::Builder *builder,
      bool is_left);
  static flatbuffers::Offset<frc971::MultiSpline> FarRocketToHPTest(
      aos::Sender<frc971::control_loops::drivetrain::Goal>::Builder *builder);

  static flatbuffers::Offset<frc971::MultiSpline> BasicSSpline(
      aos::Sender<frc971::control_loops::drivetrain::Goal>::Builder *builder);
  static flatbuffers::Offset<frc971::MultiSpline> StraightLine(
      aos::Sender<frc971::control_loops::drivetrain::Goal>::Builder *builder);
};

}  // namespace actors
}  // namespace y2019

#endif  // Y2019_ACTORS_AUTO_SPLINES_H_
