#include "aos/crio/motor_server/SensorOutput.h"

namespace aos {

SEM_ID SensorOutputs::lock_ = semBCreate(SEM_Q_PRIORITY, SEM_FULL);
std::vector<SensorOutputs *> SensorOutputs::outputs_running_;

void SensorOutputs::UpdateAll() {
  semTake(lock_, WAIT_FOREVER);
  for (auto it = outputs_running_.begin(); it != outputs_running_.end(); ++it) {
    (*it)->Update();
  }
  semGive(lock_);
}

} // namespace aos

