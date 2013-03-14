#ifndef FRC971_CONTROL_LOOPS_ANGLE_ADJUST_ANGLE_ADJUST_MOTOR_PLANT_H_
#define FRC971_CONTROL_LOOPS_ANGLE_ADJUST_ANGLE_ADJUST_MOTOR_PLANT_H_

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

StateFeedbackPlantCoefficients<2, 1, 1> MakeAngleAdjustPlantCoefficients();

StateFeedbackController<2, 1, 1> MakeAngleAdjustController();

StateFeedbackPlant<2, 1, 1> MakeAngleAdjustPlant();

StateFeedbackLoop<2, 1, 1> MakeAngleAdjustLoop();

}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_ANGLE_ADJUST_ANGLE_ADJUST_MOTOR_PLANT_H_
