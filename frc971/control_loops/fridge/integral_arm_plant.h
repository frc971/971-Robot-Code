#ifndef FRC971_CONTROL_LOOPS_FRIDGE_INTEGRAL_ARM_PLANT_H_
#define FRC971_CONTROL_LOOPS_FRIDGE_INTEGRAL_ARM_PLANT_H_

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

StateFeedbackPlantCoefficients<5, 2, 2> MakeIntegralArmPlantCoefficients();

StateFeedbackController<5, 2, 2> MakeIntegralArmController();

StateFeedbackPlant<5, 2, 2> MakeIntegralArmPlant();

StateFeedbackLoop<5, 2, 2> MakeIntegralArmLoop();

}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_FRIDGE_INTEGRAL_ARM_PLANT_H_
