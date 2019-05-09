#include "y2019/actors/auto_splines.h"

#include "frc971/control_loops/control_loops.q.h"

namespace y2019 {
namespace actors {

void MaybeFlipSpline(::frc971::MultiSpline *spline, bool is_left) {
  if (!is_left) {
    for (size_t i = 0; i < spline->spline_y.size(); i++) {
      spline->spline_y[i] *= -1.0;
    }
  }
}

// Path off of level 2 to the far side of the rocket with a panel
::frc971::MultiSpline AutonomousSplines::HABToFarRocket(bool is_left) {
  ::frc971::MultiSpline spline;
  ::frc971::Constraint longitudinal_constraint;
  ::frc971::Constraint lateral_constraint;
  ::frc971::Constraint voltage_constraint;
  ::frc971::Constraint velocity_constraint;

  longitudinal_constraint.constraint_type = 1;
  longitudinal_constraint.value = 2.0;

  lateral_constraint.constraint_type = 2;
  lateral_constraint.value = 2.0;

  voltage_constraint.constraint_type = 3;
  voltage_constraint.value = 11.0;

  velocity_constraint.constraint_type = 4;
  velocity_constraint.value = 4.0;
  velocity_constraint.start_distance = 0.0;
  velocity_constraint.end_distance = 10.0;

  spline.spline_count = 2;
  spline.spline_x = {{1.14763818102, 1.66, 3.10, 4.05, 4.45, 5.11, 5.77, 6.71,
                      7.27, 7.19, 6.57}};
  spline.spline_y = {{1.30261224364, 1.30217320136, 1.39, 1.47, 1.56346705393,
                      1.69, 1.81, 1.97, 2.18, 2.84, 3.33}};

  spline.constraints = {{longitudinal_constraint, lateral_constraint,
                         voltage_constraint, velocity_constraint}};

  MaybeFlipSpline(&spline, is_left);
  return spline;
}

// Path from the far side of the rocket to the loading station to pickup
::frc971::MultiSpline AutonomousSplines::FarRocketToHP(bool is_left) {
  ::frc971::MultiSpline spline;
  ::frc971::Constraint longitudinal_constraint;
  ::frc971::Constraint lateral_constraint;
  ::frc971::Constraint voltage_constraint;
  ::frc971::Constraint velocity_constraint;

  longitudinal_constraint.constraint_type = 1;
  longitudinal_constraint.value = 3.0;

  lateral_constraint.constraint_type = 2;
  lateral_constraint.value = 2.0;

  voltage_constraint.constraint_type = 3;
  voltage_constraint.value = 11.0;

  velocity_constraint.constraint_type = 4;
  velocity_constraint.value = 4.5;
  velocity_constraint.start_distance = 0.0;
  velocity_constraint.end_distance = 10.0;

  spline.spline_count = 1;
  spline.spline_x = {{6.6, 7.511, 6.332, 4.590, 1.561, 0.179}};
  spline.spline_y = {{3.391, 2.826, 1.384, 3.395 - 0.20, 3.429 - 0.20, 3.434 - 0.20}};

  spline.constraints = {{longitudinal_constraint, lateral_constraint,
                         voltage_constraint, velocity_constraint}};

  MaybeFlipSpline(&spline, is_left);
  return spline;
}

// Path from the human player station to the far side of the rocket with a panel
::frc971::MultiSpline AutonomousSplines::HPToFarRocket(bool is_left) {
  ::frc971::MultiSpline spline;
  ::frc971::Constraint lateral_constraint;
  ::frc971::Constraint longitudinal_constraint;
  ::frc971::Constraint velocity_constraint;
  ::frc971::Constraint voltage_constraint;

  longitudinal_constraint.constraint_type = 1;
  longitudinal_constraint.value = 3.0;

  lateral_constraint.constraint_type = 2;
  lateral_constraint.value = 3.0;

  voltage_constraint.constraint_type = 3;
  voltage_constraint.value = 11.0;

  velocity_constraint.constraint_type = 4;
  velocity_constraint.value = 2.0;
  velocity_constraint.start_distance = 7.0;
  velocity_constraint.end_distance = 15.0;

  spline.spline_count = 1;
  spline.spline_x = {{0.895115737979, 2.9155615909, 5.02361983866,
                      6.40346237218, 7.1260656844, 7.83907559509}};
  spline.spline_y = {{3.43030859063, 3.44230565037, 2.8824369646, 2.81000389973,
                      3.08853311072, 2.6933085577}};

  spline.constraints = {{lateral_constraint, velocity_constraint,
                         voltage_constraint, longitudinal_constraint}};

  MaybeFlipSpline(&spline, is_left);
  return spline;
}

// Path from the far side of the rocket to close to the loading station
::frc971::MultiSpline AutonomousSplines::FarRocketToNearHP(bool is_left) {
  ::frc971::MultiSpline spline;
  ::frc971::Constraint constraints;

  // TODO(theo): Add some real constraints.
  constraints.constraint_type = 0;
  constraints.value = 0;
  constraints.start_distance = 0;
  constraints.end_distance = 0;

  spline.spline_count = 1;
  spline.spline_x = {{6.51652191988, 6.83156293562, 5.74513904409, 2.2337653586,
                      1.94766705864, 0.727526876557}};
  spline.spline_y = {{3.2465107468, 2.88277456846, 1.93458779243, 3.44064777429,
                      3.44377880106, 3.43326367284}};

  spline.constraints = {{constraints}};

  MaybeFlipSpline(&spline, is_left);
  return spline;
}

// Path from level 2 to 2nd cargo ship bay with a hatch panel
::frc971::MultiSpline AutonomousSplines::HABToSecondCargoShipBay(bool is_left) {
  ::frc971::MultiSpline spline;
  ::frc971::Constraint constraints;
  ::frc971::Constraint longitudinal_constraint;
  ::frc971::Constraint voltage_constraint;

  constraints.constraint_type = 4;
  constraints.value = 1.6;
  constraints.start_distance = 4.0;
  constraints.end_distance = 10.0;

  longitudinal_constraint.constraint_type = 1;
  longitudinal_constraint.value = 2.0;

  voltage_constraint.constraint_type = 3;
  voltage_constraint.value = 10.0;

  spline.spline_count = 1;
  constexpr double kLess = 0.06;
  spline.spline_x = {{1.0, 2.53944573074, 5.75526086906, 6.52583747973 - kLess,
                      7.12318661548 - kLess, 7.22595029399 - kLess}};
  spline.spline_y = {{1.5, 1.48, 2.05178220103,
                      2.56666687655, 1.79340280288, 1.16170693058}};

  spline.constraints = {{constraints, longitudinal_constraint, voltage_constraint}};

  MaybeFlipSpline(&spline, is_left);
  return spline;
}

// Path from 2nd cargo ship bay to loading station
::frc971::MultiSpline AutonomousSplines::SecondCargoShipBayToHP(bool is_left) {
  ::frc971::MultiSpline spline;
  ::frc971::Constraint constraints;
  ::frc971::Constraint voltage_constraint;

  constraints.constraint_type = 4;
  constraints.value = 4.0;
  constraints.start_distance = 0;
  constraints.end_distance = 10;

  voltage_constraint.constraint_type = 3;
  voltage_constraint.value = 11.0;

  spline.spline_count = 1;
  spline.spline_x = {{7.22595029399, 7.1892447864, 6.5373977907, 5.55997590982,
                      1.22953437637, 0.32521840905}};
  constexpr double kYShift = 0.1;
  spline.spline_y = {{1.2, 1.44543230529, 2.00646674662,
                      3.43762336271 - kYShift, 3.44125430793 - kYShift,
                      3.4360348159 - kYShift}};

  spline.constraints = {{constraints, voltage_constraint}};

  MaybeFlipSpline(&spline, is_left);
  return spline;
}

// Path from loading station to 3rd cargo ship bay with a hatch panel
::frc971::MultiSpline AutonomousSplines::HPToThirdCargoShipBay(bool is_left) {
  ::frc971::MultiSpline spline;
  ::frc971::Constraint velocity_constraint;
  ::frc971::Constraint voltage_constraint;
  ::frc971::Constraint velocity_constraint2;
  velocity_constraint.constraint_type = 4;
  velocity_constraint.value = 3.5;
  velocity_constraint.start_distance = 0;
  velocity_constraint.end_distance = 10;

  velocity_constraint2.constraint_type = 4;
  velocity_constraint2.value = 2.0;
  velocity_constraint2.start_distance = 6;
  velocity_constraint2.end_distance = 10;

  voltage_constraint.constraint_type = 3;
  voltage_constraint.value = 10.0;

  spline.spline_count = 1;
  constexpr double kEndMove = 0.25;
  spline.spline_x = {{0.75, 1.112, 5.576, 7.497 - kEndMove, 7.675 - kEndMove,
                      7.768 - kEndMove}};
  spline.spline_y = {{3.431, 3.434, 2.712, 2.874, 1.786, 1.168}};

  spline.constraints = {
      {velocity_constraint, voltage_constraint, velocity_constraint2}};

  MaybeFlipSpline(&spline, is_left);
  return spline;
}

// Path from 3rd cargo ship bay to near the loading station
::frc971::MultiSpline AutonomousSplines::ThirdCargoShipBayToNearHP(
    bool is_left) {
  ::frc971::MultiSpline spline;
  ::frc971::Constraint velocity_constraint;

  velocity_constraint.constraint_type = 4;
  velocity_constraint.value = 0.5;
  velocity_constraint.start_distance = 0;
  velocity_constraint.end_distance = 10;

  spline.spline_count = 1;
  spline.spline_x = {{7.75823205276, 7.58356294646, 5.95536035287,
                      2.12377989323, 1.29347361128, 0.598613577531}};
  spline.spline_y = {{1.16791407107, 1.94564064915, 2.54565614767,
                      3.43728005786, 3.43775494434, 3.43119598027}};

  spline.constraints = {{velocity_constraint}};

  MaybeFlipSpline(&spline, is_left);
  return spline;
}

::frc971::MultiSpline AutonomousSplines::HabToFarRocketTest(bool is_left) {
  ::frc971::MultiSpline spline;
  ::frc971::Constraint longitudinal_constraint;
  ::frc971::Constraint lateral_constraint;
  ::frc971::Constraint voltage_constraint;
  ::frc971::Constraint velocity_constraint;

  longitudinal_constraint.constraint_type = 1;
  longitudinal_constraint.value = 2.0;

  lateral_constraint.constraint_type = 2;
  lateral_constraint.value = 2.0;

  voltage_constraint.constraint_type = 3;
  voltage_constraint.value = 11.0;

  velocity_constraint.constraint_type = 4;
  velocity_constraint.value = 1.7;
  velocity_constraint.start_distance = 0.0;
  velocity_constraint.end_distance = 0.8;

  spline.spline_count = 1;
  spline.spline_x = {{1.14763818102, 2.53944573074, 3.74586892131,
                      5.22352745444, 6.70255737219, 7.35784750785}};
  spline.spline_y = {{1.30261224364, 1.28295363394, 1.27450357714,
                      2.89953366429, 3.10734391012, 2.90125929705}};

  spline.constraints = {{longitudinal_constraint, lateral_constraint,
                         voltage_constraint, velocity_constraint}};

  MaybeFlipSpline(&spline, is_left);
  return spline;
}

::frc971::MultiSpline AutonomousSplines::FarRocketToHPTest() {
  ::frc971::MultiSpline spline;
  ::frc971::Constraint longitudinal_constraint;
  ::frc971::Constraint lateral_constraint;
  ::frc971::Constraint voltage_constraint;
  ::frc971::Constraint velocity_constraint;

  longitudinal_constraint.constraint_type = 1;
  longitudinal_constraint.value = 1.5;

  lateral_constraint.constraint_type = 2;
  lateral_constraint.value = 1.0;

  voltage_constraint.constraint_type = 3;
  voltage_constraint.value = 11.0;

  velocity_constraint.constraint_type = 4;
  velocity_constraint.value = 0.5;
  velocity_constraint.start_distance = 9.5;
  velocity_constraint.end_distance = 10.0;

  spline.spline_count = 1;
  spline.spline_x = {{6.53, 7.8, 7.8, 4.0, 2.0, 0.4}};
  spline.spline_y = {{3.47, 3.0, 1.5, 3.0, 3.4, 3.4}};

  spline.constraints = {{longitudinal_constraint, lateral_constraint,
                         voltage_constraint, velocity_constraint}};
  return spline;
}

::frc971::MultiSpline AutonomousSplines::HPToNearRocketTest() {
  ::frc971::MultiSpline spline;
  ::frc971::Constraint longitudinal_constraint;
  ::frc971::Constraint lateral_constraint;
  ::frc971::Constraint voltage_constraint;
  ::frc971::Constraint velocity_constraint;

  longitudinal_constraint.constraint_type = 1;
  longitudinal_constraint.value = 1.0;

  lateral_constraint.constraint_type = 2;
  lateral_constraint.value = 1.0;

  voltage_constraint.constraint_type = 3;
  voltage_constraint.value = 11.0;

  velocity_constraint.constraint_type = 4;
  velocity_constraint.value = 0.5;
  velocity_constraint.start_distance = 2.7;
  velocity_constraint.end_distance = 10.0;

  spline.spline_count = 1;
  spline.spline_x = {{1.5, 2.0, 3.0, 4.0, 4.5, 5.12}};
  spline.spline_y = {{3.4, 3.4, 3.4, 3.0, 3.0, 3.43}};

  spline.constraints = {{longitudinal_constraint, lateral_constraint,
                         voltage_constraint, velocity_constraint}};
  return spline;
}

::frc971::MultiSpline AutonomousSplines::BasicSSpline() {
  ::frc971::MultiSpline spline;
  ::frc971::Constraint longitudinal_constraint;
  ::frc971::Constraint lateral_constraint;
  ::frc971::Constraint voltage_constraint;

  longitudinal_constraint.constraint_type = 1;
  longitudinal_constraint.value = 1.0;

  lateral_constraint.constraint_type = 2;
  lateral_constraint.value = 1.0;

  voltage_constraint.constraint_type = 3;
  voltage_constraint.value = 6.0;

  spline.spline_count = 1;
  const float startx = 0.4;
  const float starty = 3.4;
  spline.spline_x = {{0.0f + startx, 0.6f + startx, 0.6f + startx,
                      0.4f + startx, 0.4f + startx, 1.0f + startx}};
  spline.spline_y = {{starty - 0.0f, starty - 0.0f, starty - 0.3f,
                      starty - 0.7f, starty - 1.0f, starty - 1.0f}};

  spline.constraints = {
      {longitudinal_constraint, lateral_constraint, voltage_constraint}};

  return spline;
}

::frc971::MultiSpline AutonomousSplines::StraightLine() {
  ::frc971::MultiSpline spline;
  ::frc971::Constraint contraints;

  contraints.constraint_type = 0;
  contraints.value = 0.0;
  contraints.start_distance = 0.0;
  contraints.end_distance = 0.0;

  spline.spline_count = 1;
  spline.spline_x = {{-12.3, -11.9, -11.5, -11.1, -10.6, -10.0}};
  spline.spline_y = {{1.25, 1.25, 1.25, 1.25, 1.25, 1.25}};

  spline.constraints = {{contraints}};

  return spline;
}

}  // namespace actors
}  // namespace y2019
