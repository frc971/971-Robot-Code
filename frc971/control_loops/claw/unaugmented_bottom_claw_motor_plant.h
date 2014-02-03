#ifndef FRC971_CONTROL_LOOPS_CLAW_UNAUGMENTED_BOTTOM_CLAW_MOTOR_PLANT_H_
#define FRC971_CONTROL_LOOPS_CLAW_UNAUGMENTED_BOTTOM_CLAW_MOTOR_PLANT_H_

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

StateFeedbackPlantCoefficients<2, 1, 1> MakeRawBottomClawPlantCoefficients();

StateFeedbackController<2, 1, 1> MakeRawBottomClawController();

StateFeedbackPlant<2, 1, 1> MakeRawBottomClawPlant();

StateFeedbackLoop<2, 1, 1> MakeRawBottomClawLoop();

}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_CLAW_UNAUGMENTED_BOTTOM_CLAW_MOTOR_PLANT_H_
