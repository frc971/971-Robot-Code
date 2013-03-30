#include "aos/common/libstdc++/memory"

#include "aos/crio/controls/ControlsManager.h"
//#include "aos/crio/motor_server/crio_control_loop_runner.h"
#include "aos/crio/motor_server/MotorServer.h"
//#include "aos/common/sensors/sensor_broadcaster.h"
#include "aos/crio/shared_libs/interrupt_bridge.h"

//#include "frc971/queues/sensor_values.h"
//#include "frc971/input/sensor_packer.h"
//#include "frc971/input/sensor_unpacker.h"

using ::std::unique_ptr;

namespace frc971 {

class MyRobot : public ::aos::crio::ControlsManager {
 public:
  MyRobot() {}

 private:
  virtual void CreateObjects() {
#if 0
    packer_ = unique_ptr< ::frc971::SensorPacker>( new ::frc971::SensorPacker());
    unpacker_ = unique_ptr< ::frc971::SensorUnpacker>( new ::frc971::SensorUnpacker());
    broadcaster_ = unique_ptr< ::aos::sensors::SensorBroadcaster< ::frc971::sensor_values>>(
        new ::aos::sensors::SensorBroadcaster< ::frc971::sensor_values>(packer_.get()));
    control_loop_runner_ = unique_ptr< ::aos::crio::CRIOControlLoopRunner< ::frc971::sensor_values>>(new ::aos::crio::CRIOControlLoopRunner< ::frc971::sensor_values>(broadcaster_.get(), unpacker_.get()));
#endif
    motor_server_notifier_ =
        unique_ptr< ::aos::crio::WDInterruptNotifier<void>>(
            new ::aos::crio::WDInterruptNotifier<void>(WriteOutputs));
  }

  virtual void RegisterControlLoops() {
    //control_loop_runner_->AddControlLoop(&shooter_);
  }

  virtual void StartSensorBroadcasters() {
    //broadcaster_->Start();
    motor_server_notifier_->StartPeriodic(0.01);
  }

  static void WriteOutputs(void *) {
    ::aos::crio::MotorServer::WriteOutputs();
  }

#if 0
  unique_ptr< ::frc971::SensorPacker> packer_;
  unique_ptr< ::frc971::SensorUnpacker> unpacker_;
  unique_ptr< ::aos::sensors::SensorBroadcaster< ::frc971::sensor_values>> broadcaster_;
  unique_ptr< ::aos::crio::CRIOControlLoopRunner< ::frc971::sensor_values>>
      control_loop_runner_;
#endif
  unique_ptr< ::aos::crio::WDInterruptNotifier<void>> motor_server_notifier_;
};

}  // namespace frc971

START_ROBOT_CLASS(::frc971::MyRobot);
