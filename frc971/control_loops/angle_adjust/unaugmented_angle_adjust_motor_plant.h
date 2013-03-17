#ifndef FRC971_CONTROL_LOOPS_ANGLE_ADJUST_UNAUGMENTED_ANGLE_ADJUST_MOTOR_PLANT_H_
#define FRC971_CONTROL_LOOPS_ANGLE_ADJUST_UNAUGMENTED_ANGLE_ADJUST_MOTOR_PLANT_H_

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

StateFeedbackPlantCoefficients<2, 1, 1> MakeAngleAdjustRawPlantCoefficients();

StateFeedbackController<2, 1, 1> MakeAngleAdjustRawController();

StateFeedbackPlant<2, 1, 1> MakeRawAngleAdjustPlant();

StateFeedbackLoop<2, 1, 1> MakeRawAngleAdjustLoop();

}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_ANGLE_ADJUST_UNAUGMENTED_ANGLE_ADJUST_MOTOR_PLANT_H_
