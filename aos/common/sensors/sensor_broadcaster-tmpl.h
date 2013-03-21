#include "aos/common/Configuration.h"

namespace aos {
namespace sensors {

template<class Values>
SensorBroadcaster<Values>::SensorBroadcaster(
    SensorPackerInterface<Values> *packer)
    : packer_(packer),
      notifier_(StaticNotify, this),
      socket_(NetworkPort::kSensors,
              configuration::GetIPAddress(
                  configuration::NetworkDevice::kAtom)),
      crio_control_loop_runner_(NULL) {
  static_assert(shm_ok<SensorData<Values>>::value,
                "it is going to get sent over a socket");
  data_.count = 0;
}

template<class Values>
void SensorBroadcaster<Values>::Start() {
  notifier_.StartPeriodic(kSensorSendFrequency);
  if (!notifier_.IsExact()) {
    LOG(FATAL, "bad choice for kSensorSendFrequency\n");
  }
}

template<class Values>
void SensorBroadcaster<Values>::RegisterControlLoopRunner(
    SensorSinkInterface<Values> *crio_control_loop_runner) {
  if (crio_control_loop_runner_ != NULL) {
    LOG(FATAL, "trying to register loop runner %p but already have %p\n",
    crio_control_loop_runner, crio_control_loop_runner_);
  }
  crio_control_loop_runner_ = crio_control_loop_runner;
}

template<class Values>
void SensorBroadcaster<Values>::Notify() {
  packer_->PackInto(&data_.values);
  data_.FillinChecksum();
  socket_.Send(&data_, sizeof(data_));
  ++data_.count;

  if ((data_.count % kSendsPerCycle) == 0) {
    if (crio_control_loop_runner_ != NULL) {
      crio_control_loop_runner_->Process(&data_);
    }
  }
}

}  // namespace sensors
}  // namespace aos
