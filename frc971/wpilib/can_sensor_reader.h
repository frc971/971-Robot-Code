#ifndef FRC971_WPILIB_CAN_SENSOR_READER_H_
#define FRC971_WPILIB_CAN_SENSOR_READER_H_

#include <vector>

#include "aos/containers/sized_array.h"
#include "aos/events/event_loop.h"
#include "aos/events/shm_event_loop.h"
#include "aos/realtime.h"
#include "frc971/wpilib/talonfx.h"

namespace frc971::wpilib {
class CANSensorReader {
 public:
  CANSensorReader(
      aos::EventLoop *event_loop,
      std::vector<ctre::phoenix6::BaseStatusSignal *> signals_registry,
      std::vector<std::shared_ptr<TalonFX>> talonfxs,
      std::function<void(ctre::phoenix::StatusCode status)>
          flatbuffer_callback);

 private:
  void Loop();

  aos::EventLoop *event_loop_;

  const std::vector<ctre::phoenix6::BaseStatusSignal *> signals_;

  // This is a vector of talonfxs becuase we don't need to care
  // about talonfxs individually.
  std::vector<std::shared_ptr<TalonFX>> talonfxs_;

  // Pointer to the timer handler used to modify the wakeup.
  ::aos::TimerHandler *timer_handler_;

  // Callback used to send the CANPosition flatbuffer
  std::function<void(ctre::phoenix::StatusCode status)> flatbuffer_callback_;
};
}  // namespace frc971::wpilib
#endif  // FRC971_WPILIB_CAN_SENSOR_READER_H_
