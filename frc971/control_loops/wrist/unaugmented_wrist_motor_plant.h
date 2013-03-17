#ifndef FRC971_CONTROL_LOOPS_WRIST_UNAUGMENTED_WRIST_MOTOR_PLANT_H_
#define FRC971_CONTROL_LOOPS_WRIST_UNAUGMENTED_WRIST_MOTOR_PLANT_H_

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

StateFeedbackPlantCoefficients<2, 1, 1> MakeRawWristPlantCoefficients();

StateFeedbackController<2, 1, 1> MakeRawWristController();

StateFeedbackPlant<2, 1, 1> MakeRawWristPlant();

StateFeedbackLoop<2, 1, 1> MakeRawWristLoop();

}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_WRIST_UNAUGMENTED_WRIST_MOTOR_PLANT_H_
