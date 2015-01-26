#ifndef FRC971_CONTROL_LOOPS_FRIDGE_ELEVATOR_MOTOR_PLANT_H_
#define FRC971_CONTROL_LOOPS_FRIDGE_ELEVATOR_MOTOR_PLANT_H_

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

StateFeedbackPlant<2, 1, 1> MakeElevatorPlant();

StateFeedbackLoop<2, 1, 1> MakeElevatorLoop();

}  // namespace frc971
}  // namespace control_loops

#endif  // FRC971_CONTROL_LOOPS_FRIDGE_ELEVATOR_MOTOR_PLANT_H_
