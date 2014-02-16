#ifndef FRC971_CONTROL_LOOPS_CLAW_UNAUGMENTED_TOP_CLAW_MOTOR_PLANT_H_
#define FRC971_CONTROL_LOOPS_CLAW_UNAUGMENTED_TOP_CLAW_MOTOR_PLANT_H_

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

StateFeedbackPlantCoefficients<2, 1, 1> MakeRawTopClawPlantCoefficients();

StateFeedbackController<2, 1, 1> MakeRawTopClawController();

StateFeedbackPlant<2, 1, 1> MakeRawTopClawPlant();

StateFeedbackLoop<2, 1, 1> MakeRawTopClawLoop();

}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_CLAW_UNAUGMENTED_TOP_CLAW_MOTOR_PLANT_H_
