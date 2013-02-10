#include "aos/crio/controls/ControlsManager.h"

#include "aos/crio/motor_server/CRIOControlLoopRunner.h"

namespace frc971 {

class MyRobot : public ::aos::crio::ControlsManager {
 public:
  virtual void RegisterControlLoops() {
    //::aos::crio::CRIOControlLoopRunner::AddControlLoop(&shooter_);
  }

 private:
  //::frc971::control_loops::ShooterMotor shooter_;
};

}  // namespace frc971

START_ROBOT_CLASS(::frc971::MyRobot);
