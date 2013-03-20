#include "aos/common/Configuration.h"
#include "aos/common/inttypes.h"

namespace aos {
namespace sensors {

template<class Values>
const time::Time SensorReceiver<Values>::kJitterDelay =
    time::Time::InSeconds(0.0035);
// Not a multiple of kSensorSendFrequency to unwedge ourself if we hit some bug
// where it needs to get off of that frequency to work.
template<class Values>
const time::Time SensorReceiver<Values>::kGiveupTime =
    time::Time::InSeconds(0.1555);

template<class Values>
SensorReceiver<Values>::SensorReceiver(
    SensorUnpackerInterface<Values> *unpacker)
    : unpacker_(unpacker),
      start_time_(0, 0),
      last_good_time_(0, 0) {
  Unsynchronize();
}

template<class Values>
void SensorReceiver<Values>::RunIteration() {
  if (synchronized_) {
    if (ReceiveData()) {
      LOG(DEBUG, "receive said to try a reset\n");
      Unsynchronize();
    } else {
      if (GoodPacket()) {
        unpacker_->UnpackFrom(&data_.values);
        last_good_time_ = time::Time::Now();
      } else {
        if ((time::Time::Now() - last_good_time_) > kGiveupTime) {
          LOG(INFO, "resetting because didn't get a good one in too long\n");
          Unsynchronize();
        } else {
          // We got a packet, but it wasn't an interesting one.
        }
      }
    }
  } else {
    LOG(INFO, "resetting to try receiving data\n");
    Reset();
    if (Synchronize()) {
      LOG(INFO, "synchronized successfully\n");
      synchronized_ = true;
      before_better_cycles_ = after_better_cycles_ = 0;
      last_good_time_ = time::Time::Now();
    } else {
      LOG(INFO, "synchronization failed\n");
    }
  }
}

template<class Values>
bool SensorReceiver<Values>::GoodPacket() {
  bool good;
  // If it's a multiple of kSensorSendFrequency from start_count_.
  if (((data_.count - start_count_) % kSendsPerCycle) == 0) {
    if (((data_.count - start_count_) / kSendsPerCycle) ==
        ((NextLoopTime() - start_time_).ToNSec() / kLoopFrequency.ToNSec())) {
      good = true;
    } else {
      LOG(INFO, "packet %"PRId32" late. is packet #%d, wanted #%"PRId64" now\n",
          data_.count, (data_.count - start_count_) / kSendsPerCycle,
          (NextLoopTime() - start_time_).ToNSec() / kLoopFrequency.ToNSec());
      good = false;
    }
  } else {
    good = false;
  }

  static time::Time last_time(0, 0);
  time::Time now = time::Time::Now();
  time::Time next_goal_time = NextLoopTime() - kJitterDelay;
  // If this is the packet after the right one.
  if (((data_.count - start_count_ - 1) % kSendsPerCycle) == 0) {
    // If this one is much closer than the last one (aka the one that we used).
    if ((now - next_goal_time).abs() * 11 / 10 <
        (last_time - next_goal_time).abs()) {
      LOG(DEBUG, "next one better than one being used %d\n",
          after_better_cycles_);
      if (after_better_cycles_ > kBadCyclesToSwitch) {
        LOG(INFO, "switching to the packet after\n");
        UpdateStartTime(data_.count);
        before_better_cycles_ = after_better_cycles_ = 0;
      } else {
        ++after_better_cycles_;
      }
    } else {
      after_better_cycles_ = 0;
    }
  }
  // If this is the right packet.
  if (((data_.count - start_count_) % kSendsPerCycle) == 0) {
    // If the last one was closer than this one (aka the one that we used).
    if ((last_time - next_goal_time).abs() * 11 / 10 <
        (now - next_goal_time).abs()) {
      LOG(DEBUG, "previous better than one being used %d\n",
          before_better_cycles_);
      if (before_better_cycles_ > kBadCyclesToSwitch) {
        LOG(INFO, "switching to the packet before\n");
        UpdateStartTime(data_.count - 1);
        before_better_cycles_ = after_better_cycles_ = 0;
      } else {
        ++before_better_cycles_;
      }
    } else {
      before_better_cycles_ = 0;
    }
  }
  last_time = now;

  return good;
}

template<class Values>
void SensorReceiver<Values>::UpdateStartTime(int new_start_count) {
  start_time_ += kSensorSendFrequency * (new_start_count - start_count_ - 1);
  start_count_ = new_start_count;
}

// Looks for when the timestamps transition from before where we want to after
// and then picks whichever one was closer. After that, reads kTestCycles and
// makes sure that at most 1 is bad.
template<class Values>
bool SensorReceiver<Values>::Synchronize() {
  time::Time old_received_time(0, 0);
  const time::Time start_time = time::Time::Now();
  // When we want to send out the next set of values.
  time::Time goal_time = NextLoopTime(start_time) - kJitterDelay;
  if (goal_time < start_time) {
    goal_time += kLoopFrequency;
  }
  while (true) {
    if (ReceiveData()) return false;
    time::Time received_time = time::Time::Now();
    if (received_time >= goal_time) {
      // If this was the very first one we got, try again.
      if (old_received_time == time::Time(0, 0)){
        LOG(INFO, "first one we got was too late\n");
        return false;
      }

      assert(old_received_time < goal_time);

      // If the most recent one is closer than the last one.
      if ((received_time - goal_time).abs() <
          (old_received_time - goal_time).abs()) {
        start_count_ = data_.count;
      } else {
        start_count_ = data_.count - 1;
      }
      start_time_ = goal_time;

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
            LOG(INFO, "rejected time of the last good packet. "
                "got %"PRId32"s%"PRId32"ns. wanted %"PRId32"s%"PRId32"ns\n",
                received_time.sec(), received_time.nsec(),
                goal_time.sec(), goal_time.nsec());
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
void SensorReceiver<Values>::Unsynchronize() {
  synchronized_ = false;
  before_better_cycles_ = after_better_cycles_ = 0;
}

template<class Values>
const time::Time NetworkSensorReceiver<Values>::kWarmupTime =
    time::Time::InSeconds(0.075);

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
