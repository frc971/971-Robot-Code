#ifndef Y2015_BOT3_CONTROL_LOOPS_ELEVATOR_ELEVATOR_MOTOR_PLANT_H_
#define Y2015_BOT3_CONTROL_LOOPS_ELEVATOR_ELEVATOR_MOTOR_PLANT_H_

#include "frc971/control_loops/state_feedback_loop.h"

namespace y2015_bot3 {
namespace control_loops {

StateFeedbackPlantCoefficients<2, 1, 1> MakeElevatorPlantCoefficients();

StateFeedbackController<2, 1, 1> MakeElevatorController();

StateFeedbackPlant<2, 1, 1> MakeElevatorPlant();

StateFeedbackLoop<2, 1, 1> MakeElevatorLoop();

}  // namespace control_loops
}  // namespace y2015_bot3

#endif  // Y2015_BOT3_CONTROL_LOOPS_ELEVATOR_ELEVATOR_MOTOR_PLANT_H_
