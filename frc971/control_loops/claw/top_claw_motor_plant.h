#ifndef FRC971_CONTROL_LOOPS_CLAW_TOP_CLAW_MOTOR_PLANT_H_
#define FRC971_CONTROL_LOOPS_CLAW_TOP_CLAW_MOTOR_PLANT_H_

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

StateFeedbackPlantCoefficients<3, 1, 1> MakeTopClawPlantCoefficients();

StateFeedbackController<3, 1, 1> MakeTopClawController();

StateFeedbackPlant<3, 1, 1> MakeTopClawPlant();

StateFeedbackLoop<3, 1, 1> MakeTopClawLoop();

}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_CLAW_TOP_CLAW_MOTOR_PLANT_H_
