#include "MotorOutput.h"

namespace aos {

SEM_ID MotorOutput::lock = semBCreate(SEM_Q_PRIORITY, SEM_FULL);
std::vector<MotorOutput *> MotorOutput::instances;

void MotorOutput::Run() {
  semTake(lock, WAIT_FOREVER);
  instances.push_back(this);
  semGive(lock);
}
void MotorOutput::RunIterationAll() {
  semTake(lock, WAIT_FOREVER);
  for (auto it = instances.begin(); it != instances.end(); ++it) {
    (*it)->RunIteration();
  }
  semGive(lock);
}

} // namespace aos

