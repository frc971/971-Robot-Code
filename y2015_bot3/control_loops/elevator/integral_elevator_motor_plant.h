#ifndef Y2015_BOT3_CONTROL_LOOPS_ELEVATOR_INTEGRAL_ELEVATOR_MOTOR_PLANT_H_
#define Y2015_BOT3_CONTROL_LOOPS_ELEVATOR_INTEGRAL_ELEVATOR_MOTOR_PLANT_H_

#include "frc971/control_loops/state_feedback_loop.h"

namespace y2015_bot3 {
namespace control_loops {

StateFeedbackPlantCoefficients<3, 1, 1> MakeIntegralElevatorPlantCoefficients();

StateFeedbackController<3, 1, 1> MakeIntegralElevatorController();

StateFeedbackPlant<3, 1, 1> MakeIntegralElevatorPlant();

StateFeedbackLoop<3, 1, 1> MakeIntegralElevatorLoop();

}  // namespace control_loops
}  // namespace y2015_bot3

#endif  // Y2015_BOT3_CONTROL_LOOPS_ELEVATOR_INTEGRAL_ELEVATOR_MOTOR_PLANT_H_
