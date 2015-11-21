#ifndef Y2015_BOT3_CONTROL_LOOPS_DRIVETRAIN_POLYDRIVETRAIN_CIM_PLANT_H_
#define Y2015_BOT3_CONTROL_LOOPS_DRIVETRAIN_POLYDRIVETRAIN_CIM_PLANT_H_

#include "frc971/control_loops/state_feedback_loop.h"

namespace y2015_bot3 {
namespace control_loops {

StateFeedbackPlantCoefficients<1, 1, 1> MakeCIMPlantCoefficients();

StateFeedbackController<1, 1, 1> MakeCIMController();

StateFeedbackPlant<1, 1, 1> MakeCIMPlant();

StateFeedbackLoop<1, 1, 1> MakeCIMLoop();

}  // namespace control_loops
}  // namespace y2015_bot3

#endif  // Y2015_BOT3_CONTROL_LOOPS_DRIVETRAIN_POLYDRIVETRAIN_CIM_PLANT_H_
