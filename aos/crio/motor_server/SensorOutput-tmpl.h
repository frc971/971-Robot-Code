#include "aos/common/input/SensorInput.h"

namespace aos {

template<class Values> std::vector<SensorOutput<Values> *> SensorOutput<Values>::output_running_;
template<class Values> void SensorOutput<Values>::Run() {
  semTake(lock_, WAIT_FOREVER);
  output_running_.push_back(this);
  outputs_running_.push_back(this);
  semGive(lock_);
}

template<class Values> void SensorOutput<Values>::RunIterationAll(Values &vals) {
  semTake(lock_, WAIT_FOREVER);
  for (auto it = output_running_.begin(); it != output_running_.end(); ++it) {
    (*it)->RunIteration(vals);
  }
  semGive(lock_);
}
template<class Values> void SensorOutput<Values>::Update() {
  Values vals;
  RunIteration(vals);
  SensorInput<Values>::RunIterationAll(vals);
}

} // namespace aos

