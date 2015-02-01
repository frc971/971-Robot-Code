#include "frc971/control_loops/fridge/fridge.h"

#include "aos/common/controls/control_loops.q.h"
#include "aos/common/logging/logging.h"

#include "frc971/control_loops/fridge/elevator_motor_plant.h"
#include "frc971/control_loops/fridge/arm_motor_plant.h"

namespace frc971 {
namespace control_loops {

Fridge::Fridge(control_loops::FridgeQueue *fridge)
    : aos::controls::ControlLoop<control_loops::FridgeQueue>(fridge),
      arm_loop_(new StateFeedbackLoop<4, 2, 2>(MakeArmLoop())),
      elev_loop_(new StateFeedbackLoop<4, 2, 2>(MakeElevatorLoop())) {}

void Fridge::RunIteration(
    const control_loops::FridgeQueue::Goal * /*goal*/,
    const control_loops::FridgeQueue::Position * /*position*/,
    control_loops::FridgeQueue::Output * /*output*/,
    control_loops::FridgeQueue::Status * /*status*/) {

  LOG(DEBUG, "Hi Brian!\n");
}

}  // namespace control_loops
}  // namespace frc971
