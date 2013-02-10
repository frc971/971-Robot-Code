#include "WPILib/Victor.h"

#include "aos/crio/motor_server/MotorOutput.h"
#include "aos/aos_core.h"

namespace frc971 {

class MotorWriter : public aos::MotorOutput {
  virtual void RunIteration() {
  }
};

}  // namespace frc971

AOS_RUN(frc971::MotorWriter)
