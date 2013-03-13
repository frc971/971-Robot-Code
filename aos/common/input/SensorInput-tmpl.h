#ifndef __VXWORKS__
#include "aos/common/network/ReceiveSocket.h"
#include "aos/common/Configuration.h"
#endif
#include "aos/common/logging/logging.h"

namespace aos {

#ifdef __VXWORKS__
template<class Values> SEM_ID SensorInput<Values>::lock_ = semBCreate(SEM_Q_PRIORITY, SEM_FULL);
template<class Values> std::vector<SensorInput<Values> *> SensorInput<Values>::running_;
#endif
template<class Values> void SensorInput<Values>::Run() {
#ifndef __VXWORKS__
	ReceiveSocket sock(NetworkPort::kSensors);
  Values values;
	while (true) {
		if (sock.Recv(&values, sizeof(values)) == -1) {
      LOG(WARNING, "socket receive failed\n");
      continue;
    }
    RunIteration(values);
	}
#else
  semTake(lock_, WAIT_FOREVER);
  running_.push_back(this);
  semGive(lock_);
#endif
}

#ifdef __VXWORKS__
template<class Values> void SensorInput<Values>::RunIterationAll(Values &vals) {
  semTake(lock_, WAIT_FOREVER);
  for (auto it = running_.begin(); it != running_.end(); ++it) {
    (*it)->RunIteration(vals);
  }
  semGive(lock_);
}
#endif

} // namespace aos
