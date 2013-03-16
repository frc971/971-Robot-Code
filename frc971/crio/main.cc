#include "aos/common/libstdc++/memory"

#include "aos/crio/controls/ControlsManager.h"
#include "aos/crio/motor_server/crio_control_loop_runner.h"
#include "aos/common/sensors/sensor_broadcaster.h"

#include "frc971/queues/sensor_values.h"
#include "frc971/input/sensor_packer.h"
#include "frc971/input/sensor_unpacker.h"

using ::std::unique_ptr;

namespace frc971 {

class MyRobot : public ::aos::crio::ControlsManager {
 public:
  MyRobot() {}

  virtual void CreateObjects() {
    packer_ = unique_ptr< ::frc971::SensorPacker>( new ::frc971::SensorPacker());
    unpacker_ = unique_ptr< ::frc971::SensorUnpacker>( new ::frc971::SensorUnpacker());
    broadcaster_ = unique_ptr< ::aos::sensors::SensorBroadcaster< ::frc971::sensor_values>>(
        new ::aos::sensors::SensorBroadcaster< ::frc971::sensor_values>(packer_.get()));
    control_loop_runner_ = unique_ptr< ::aos::crio::CRIOControlLoopRunner< ::frc971::sensor_values>>(new ::aos::crio::CRIOControlLoopRunner< ::frc971::sensor_values>(broadcaster_.get(), unpacker_.get()));
  }

  virtual void RegisterControlLoops() {
    //control_loop_runner_->AddControlLoop(&shooter_);
  }

  virtual void StartSensorBroadcasters() {
    broadcaster_->Start();
  }

  unique_ptr< ::frc971::SensorPacker> packer_;
  unique_ptr< ::frc971::SensorUnpacker> unpacker_;
  unique_ptr< ::aos::sensors::SensorBroadcaster< ::frc971::sensor_values>> broadcaster_;
  unique_ptr< ::aos::crio::CRIOControlLoopRunner< ::frc971::sensor_values>>
      control_loop_runner_;
};

}  // namespace frc971

START_ROBOT_CLASS(::frc971::MyRobot);
