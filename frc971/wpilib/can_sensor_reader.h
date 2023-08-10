#ifndef FRC971_WPILIB_CAN_SENSOR_READER_H_
#define FRC971_WPILIB_CAN_SENSOR_READER_H_

#include <vector>

#include "aos/containers/sized_array.h"
#include "aos/events/event_loop.h"
#include "aos/events/shm_event_loop.h"
#include "aos/realtime.h"
#include "frc971/wpilib/falcon.h"

namespace frc971 {
namespace wpilib {
class CANSensorReader {
 public:
  CANSensorReader(
      aos::EventLoop *event_loop,
      std::vector<ctre::phoenix6::BaseStatusSignal *> signals_registry,
      std::vector<std::shared_ptr<Falcon>> falcons);

 private:
  void Loop();

  aos::EventLoop *event_loop_;

  const std::vector<ctre::phoenix6::BaseStatusSignal *> signals_;
  aos::Sender<control_loops::drivetrain::CANPosition> can_position_sender_;

  // This is a vector of falcons becuase we don't need to care
  // about falcons individually.
  std::vector<std::shared_ptr<Falcon>> falcons_;

  // Pointer to the timer handler used to modify the wakeup.
  ::aos::TimerHandler *timer_handler_;
};
}  // namespace wpilib
}  // namespace frc971
#endif  // FRC971_WPILIB_CAN_SENSOR_READER_H_
