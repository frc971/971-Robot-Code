#ifndef Y2015_CONTROL_LOOPS_CLAW_CLAW_MOTOR_PLANT_H_
#define Y2015_CONTROL_LOOPS_CLAW_CLAW_MOTOR_PLANT_H_

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

StateFeedbackPlantCoefficients<2, 1, 1> MakeClawPlantCoefficients();

StateFeedbackController<2, 1, 1> MakeClawController();

StateFeedbackPlant<2, 1, 1> MakeClawPlant();

StateFeedbackLoop<2, 1, 1> MakeClawLoop();

}  // namespace control_loops
}  // namespace frc971

#endif  // Y2015_CONTROL_LOOPS_CLAW_CLAW_MOTOR_PLANT_H_
