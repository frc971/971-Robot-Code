#ifndef FRC971_CONTROL_LOOPS_FRIDGE_ARM_MOTOR_PLANT_H_
#define FRC971_CONTROL_LOOPS_FRIDGE_ARM_MOTOR_PLANT_H_

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

StateFeedbackPlantCoefficients<4, 2, 2> MakeArmPlantCoefficients();

StateFeedbackController<4, 2, 2> MakeArmController();

StateFeedbackPlant<4, 2, 2> MakeArmPlant();

StateFeedbackLoop<4, 2, 2> MakeArmLoop();

}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_FRIDGE_ARM_MOTOR_PLANT_H_
