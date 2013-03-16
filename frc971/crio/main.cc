#include "aos/crio/controls/ControlsManager.h"
#include "aos/crio/motor_server/crio_control_loop_runner.h"
#include "aos/common/sensors/sensor_broadcaster.h"

#include "frc971/queues/sensor_values.h"
#include "frc971/input/sensor_packer.h"
#include "frc971/input/sensor_unpacker.h"

namespace frc971 {

class MyRobot : public ::aos::crio::ControlsManager {
 public:
  MyRobot()
      : broadcaster_(&packer_),
        control_loop_runner_(&broadcaster_, &unpacker_) {}

  virtual void RegisterControlLoops() {
    //control_loop_runner_.AddControlLoop(&shooter_);
  }

  virtual void StartSensorBroadcasters() {
    broadcaster_.Start();
  }

  ::frc971::SensorPacker packer_;
  ::frc971::SensorUnpacker unpacker_;
  ::aos::sensors::SensorBroadcaster< ::frc971::sensor_values> broadcaster_;
  ::aos::crio::CRIOControlLoopRunner< ::frc971::sensor_values>
      control_loop_runner_;
};

}  // namespace frc971

START_ROBOT_CLASS(::frc971::MyRobot);
