#ifndef BOT3_CONTROL_LOOPS_ELEVATOR_ELEVATOR_MOTOR_PLANT_H_
#define BOT3_CONTROL_LOOPS_ELEVATOR_ELEVATOR_MOTOR_PLANT_H_

#include "frc971/control_loops/state_feedback_loop.h"

namespace bot3 {
namespace control_loops {

StateFeedbackPlantCoefficients<2, 1, 1> MakeElevatorPlantCoefficients();

StateFeedbackController<2, 1, 1> MakeElevatorController();

StateFeedbackPlant<2, 1, 1> MakeElevatorPlant();

StateFeedbackLoop<2, 1, 1> MakeElevatorLoop();

}  // namespace control_loops
}  // namespace bot3

#endif  // BOT3_CONTROL_LOOPS_ELEVATOR_ELEVATOR_MOTOR_PLANT_H_
