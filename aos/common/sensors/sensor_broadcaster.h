#ifndef AOS_COMMON_SENSORS_SENSOR_BROADCASTER_H_
#define AOS_COMMON_SENSORS_SENSOR_BROADCASTER_H_

#include "aos/crio/shared_libs/interrupt_bridge.h"
#include "aos/common/network/SendSocket.h"
#include "aos/common/sensors/sensors.h"
#include "aos/common/sensors/sensor_packer.h"
#include "aos/common/sensors/sensor_sink.h"
#include "aos/common/macros.h"

namespace aos {
namespace crio {

template<class Values>
class CRIOControlLoopRunner;

}  // namespace crio
namespace sensors {

// A class that handles sending sensor values to the atom.
// See sensors.h for an overview of where this fits in.
template<class Values>
class SensorBroadcaster {
 public:
  // Does not take ownership of packer.
  SensorBroadcaster(SensorPackerInterface<Values> *packer);

  void Start();

 private:
  // So that it can access RegisterControlLoopRunner.
  friend class crio::CRIOControlLoopRunner<Values>;

  // Registers an object to get run on control loop timing. Only designed to be
  // called by crio::CRIOControlLoopRunner.
  // Does not take ownership of crio_control_loop_runner.
  void RegisterControlLoopRunner(
      SensorSinkInterface<Values> *crio_control_loop_runner);

  static void StaticNotify(SensorBroadcaster<Values> *self) { self->Notify(); }
  void Notify();

  SensorPackerInterface<Values> *const packer_;
  crio::WDInterruptNotifier<SensorBroadcaster<Values>> notifier_;
  SendSocket socket_;
  SensorData<Values> data_;
  SensorSinkInterface<Values> *crio_control_loop_runner_;

  DISALLOW_COPY_AND_ASSIGN(SensorBroadcaster<Values>);
};

}  // namespace sensors
}  // namespace aos

#include "sensor_broadcaster-tmpl.h"

#endif  // AOS_COMMON_SENSORS_SENSOR_BROADCASTER_H_
