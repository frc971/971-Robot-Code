#include "aos/common/logging/logging.h"

#include "aos/crio/motor_server/MotorOutput.h"
#include "aos/crio/motor_server/MotorServer.h"

namespace aos {
namespace crio {

template<class Values>
CRIOControlLoopRunner<Values>::CRIOControlLoopRunner(
    sensors::SensorBroadcaster<Values> *broadcaster,
    sensors::SensorUnpackerInterface<Values> *unpacker)
    : broadcaster_(broadcaster),
      unpacker_(unpacker) {}

template<class Values>
void CRIOControlLoopRunner<Values>::AddControlLoop(
    ::aos::control_loops::SerializableControlLoop *loop) {
  loops_.push_back(loop);
  MotorServer::RegisterControlLoopGoal(loop);
}

template<class Values>
void CRIOControlLoopRunner<Values>::Process(sensors::SensorData<Values> *data) {
  unpacker_->UnpackFrom(&data.values);
  for (auto it = loops_.begin(); it != loops_.end(); ++it) {
    (*it)->Iterate();
  }
  // TODO(brians): make these nice objects too
  MotorOutput::RunIterationAll();
  MotorServer::WriteOutputs();
}

}  // namespace crio
}  // namespace aos
