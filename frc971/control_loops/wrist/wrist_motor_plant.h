#ifndef FRC971_CONTROL_LOOPS_WRIST_WRIST_MOTOR_PLANT_H_
#define FRC971_CONTROL_LOOPS_WRIST_WRIST_MOTOR_PLANT_H_

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

StateFeedbackPlantCoefficients<3, 1, 1> MakeWristPlantCoefficients();

StateFeedbackController<3, 1, 1> MakeWristController();

StateFeedbackPlant<3, 1, 1> MakeWristPlant();

StateFeedbackLoop<3, 1, 1> MakeWristLoop();

}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_WRIST_WRIST_MOTOR_PLANT_H_
