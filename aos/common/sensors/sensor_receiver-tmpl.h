#include "aos/common/Configuration.h"
#include "aos/common/inttypes.h"

namespace aos {
namespace sensors {

template<class Values>
const time::Time SensorReceiver<Values>::kJitterDelay =
    time::Time::InSeconds(0.0015);

template<class Values>
SensorReceiver<Values>::SensorReceiver(
    SensorUnpackerInterface<Values> *unpacker)
    : unpacker_(unpacker),
      synchronized_(false) {}

template<class Values>
void SensorReceiver<Values>::RunIteration() {
  if (synchronized_) {
    if (ReceiveData()) {
      LOG(DEBUG, "receive said to try a reset\n");
      synchronized_ = false;
      return;
    }
    if (GoodPacket()) {
      unpacker_->UnpackFrom(&data_.values);
    }
  } else {
    LOG(INFO, "resetting to try receiving data\n");
    Reset();
    if (Synchronize()) {
      LOG(INFO, "synchronized successfully\n");
      synchronized_ = true;
    }
  }
}

template<class Values>
bool SensorReceiver<Values>::GoodPacket() {
  // If it's a multiple of kSensorSendFrequency from start_count_.
  if (((data_.count - start_count_) % kSendsPerCycle) == 0) {
    return true;
  } else {
    return false;
  }
}

// Looks for when the timestamps transition from before where we want to after
// and then picks whichever one was closer. After that, reads kTestCycles and
// makes sure that at most 1 is bad.
template<class Values>
bool SensorReceiver<Values>::Synchronize() {
  time::Time old_received_time(0, 0);
  time::Time start_time = time::Time::Now();
  // When we want to send out the next set of values.
  time::Time goal_time = (start_time / kLoopFrequency.ToNSec()) *
      kLoopFrequency.ToNSec() +
      kLoopFrequency - kJitterDelay;
  while (true) {
    if (ReceiveData()) return false;
    time::Time received_time = time::Time::Now();
    if (received_time > goal_time) {
      // If this was the very first one we got, try again.
      if (old_received_time == time::Time(0, 0)) return false;

      assert(old_received_time < goal_time);

      // If the most recent one is closer than the last one.
      if ((received_time - goal_time).abs() <
          (old_received_time - goal_time).abs()) {
        start_count_ = data_.count;
      } else {
        start_count_ = data_.count - 1;
      }

      int bad_count = 0;
      for (int i = 0; i < kTestCycles;) {
        ReceiveData();
        received_time = time::Time::Now();
        if (GoodPacket()) {
          LOG(DEBUG, "checking packet count=%"PRId32
              " received at %"PRId32"s%"PRId32"ns\n",
              data_.count, received_time.sec(), received_time.nsec());
          // If |the difference between the goal time for this numbered packet
          // and the time we actually got this one| is too big.
          if (((goal_time +
                kSensorSendFrequency * (data_.count - start_count_)) -
               received_time).abs() > kSensorSendFrequency) {
            LOG(INFO, "rejected time of the last good packet\n");
            ++bad_count;
          }
          ++i;
        }
        if (bad_count > 1) {
          LOG(WARNING, "got multiple packets with bad timestamps\n");
          return false;
        }
      }

      return true;
    }

    old_received_time = received_time;
  }
}

template<class Values>
bool SensorReceiver<Values>::ReceiveData() {
  int old_count = data_.count;
  DoReceiveData();
  data_.NetworkToHost();
  if (data_.count < 0) {
    LOG(FATAL, "data count overflowed. currently %"PRId32"\n", data_.count);
  }
  if (data_.count < old_count) {
    LOG(INFO, "count reset. was %"PRId32", now %"PRId32"\n",
        old_count, data_.count);
    return true;
  }
  LOG(DEBUG, "received data count %"PRId32"\n", data_.count);
  return false;
}

template<class Values>
const time::Time NetworkSensorReceiver<Values>::kWarmupTime =
    time::Time::InSeconds(0.125);

template<class Values>
NetworkSensorReceiver<Values>::NetworkSensorReceiver(
    SensorUnpackerInterface<Values> *unpacker)
    : SensorReceiver<Values>(unpacker),
      socket_(NetworkPort::kSensors) {}

template<class Values>
void NetworkSensorReceiver<Values>::Reset() {
  LOG(INFO, "beginning warm up\n");
  time::Time start = time::Time::Now();
  while ((time::Time::Now() - start) < kWarmupTime) {
    socket_.Receive(this->data(), sizeof(*this->data()));
  }
  LOG(INFO, "done warming up\n");
}

template<class Values>
void NetworkSensorReceiver<Values>::DoReceiveData() {
  while (true) {
    if (socket_.Receive(this->data(), sizeof(*this->data())) ==
        sizeof(*this->data())) {
      return;
    }
    LOG(WARNING, "received incorrect amount of data\n");
  }
}

}  // namespace sensors
}  // namespace aos
