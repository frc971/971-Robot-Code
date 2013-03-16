#ifndef AOS_CRIO_MOTOR_SERVER_CRIO_CONTROL_LOOP_RUNNER_H_
#define AOS_CRIO_MOTOR_SERVER_CRIO_CONTROL_LOOP_RUNNER_H_

#include <vector>
#include <semLib.h>

#include "aos/common/control_loop/ControlLoop.h"
#include "aos/common/mutex.h"
#include "aos/common/sensors/sensor_sink.h"
#include "aos/common/sensors/sensor_broadcaster.h"
#include "aos/common/sensors/sensor_unpacker.h"
#include "aos/common/macros.h"

namespace aos {
namespace crio {

// Instances can run 0-N control loops on the cRIO.
// See aos/common/sensors/sensors.h for an overview of where this fits in.
template<class Values>
class CRIOControlLoopRunner : public sensors::SensorSinkInterface<Values> {
 public:
  // Does not take ownership of broadcaster or unpacker.
  // *broadcaster must not be started yet.
  CRIOControlLoopRunner(sensors::SensorBroadcaster<Values> *broadcaster,
                        sensors::SensorUnpackerInterface<Values> *unpacker);

  // Adds a control loop to run.
  // Must not be called after the broadcaster is started.
  void AddControlLoop(control_loops::SerializableControlLoop *loop);

  void Process(sensors::SensorData<Values> *data);

 private:
  std::vector<control_loops::SerializableControlLoop *> loops_;
  sensors::SensorBroadcaster<Values> *const broadcaster_;
  sensors::SensorUnpackerInterface<Values> *const unpacker_;

  DISALLOW_COPY_AND_ASSIGN(CRIOControlLoopRunner<Values>);
};


}  // namespace crio
}  // namespace aos

#include "aos/crio/motor_server/crio_control_loop_runner-tmpl.h"

#endif  // AOS_CRIO_MOTOR_SERVER_CRIO_CONTROL_LOOP_RUNNER_H_
