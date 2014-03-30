#ifndef FRC971_CONTROL_LOOPS_CLAW_CLAW_MOTOR_PLANT_H_
#define FRC971_CONTROL_LOOPS_CLAW_CLAW_MOTOR_PLANT_H_

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {
static constexpr double kClawMomentOfInertiaRatio = 0.933333;


StateFeedbackPlantCoefficients<4, 2, 2> MakeClawPlantCoefficients();

StateFeedbackController<4, 2, 2> MakeClawController();

StateFeedbackPlant<4, 2, 2> MakeClawPlant();

StateFeedbackLoop<4, 2, 2> MakeClawLoop();

}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_CLAW_CLAW_MOTOR_PLANT_H_
