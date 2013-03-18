#ifndef AOS_COMMON_SENSORS_SENSOR_RECEIVER_H_
#define AOS_COMMON_SENSORS_SENSOR_RECEIVER_H_

#include "aos/common/sensors/sensor_unpacker.h"
#include "aos/common/network/ReceiveSocket.h"
#include "aos/common/sensors/sensors.h"
#include "aos/common/time.h"
#include "aos/common/gtest_prod.h"

namespace aos {
namespace sensors {
namespace testing {

FORWARD_DECLARE_TEST_CASE(SensorReceiverTest, Simple);

}  // namespace testing

// A class that handles receiving sensor values from the cRIO.
// See sensors.h for an overview of where this fits in.
//
// Abstract class to make testing the complex logic for choosing which data to
// use easier.
template<class Values>
class SensorReceiver {
 public:
  // Does not take ownership of unpacker.
  SensorReceiver(SensorUnpackerInterface<Values> *unpacker);

  void RunIteration();

 protected:
  SensorData<Values> *data() { return &data_; }

 private:
  // How long before the control loops run to aim for receiving sensor data (to
  // prevent jitter if some packets arrive up to this much later).
  static const time::Time kJitterDelay;
  // How many cycles not to send data out to make sure that we're in phase
  // (during this time, the code verifies that <= 1 cycle is not within 1
  // cycle's time of kJitterDelay).
  static const int kTestCycles = 10;
  // How many cycles that we need (consecutively) of another packet being closer
  // to the right time than the ones we're reading before we switch.
  static const int kBadCyclesToSwitch = 10;
  // If we don't get a good packet in this long, then we Synchronize() again.
  static const time::Time kGiveupTime;

  FRIEND_TEST_NAMESPACE(SensorReceiverTest, Simple, testing);

  // Subclasses need to implement this to read 1 set of data (blocking until it
  // is available) into data().
  virtual void DoReceiveData() = 0;

  // Optional: if subclasses can do anything to reinitialize after there are
  // problems, they should do it here.
  // This will be called right before calling DoReceiveData() the first time.
  virtual void Reset() {}

  // Returns whether the current packet looks like a good one to use.
  bool GoodPacket();

  // Updates start_count_ to new_start_count and changes start_time_
  // accordingly. Does it relative to avoid resetting start_time_ based off of 1
  // bad packet.
  void UpdateStartTime(int new_start_count);

  // Synchronizes with incoming packets and sets start_count_ to where we
  // started reading.
  // Returns whether it succeeded in locking on.
  bool Synchronize();
  // Receives a set of values and makes sure that it's sane.
  // Returns whether to start over again with timing.
  bool ReceiveData();
  void Unsynchronize();

  SensorUnpackerInterface<Values> *const unpacker_;
  SensorData<Values> data_;
  // The count that we started out (all other sent packets will be multiples of
  // this).
  int32_t start_count_;
  // When start_count_ "should" have been received. Used for checking to make
  // sure that we don't send out a packet late.
  time::Time start_time_;
  bool synchronized_;
  int before_better_cycles_, after_better_cycles_;
  // The time of the last packet that we sent out.
  time::Time last_good_time_;

  DISALLOW_COPY_AND_ASSIGN(SensorReceiver<Values>);
};

// A SensorReceiver that receives data from a SensorBroadcaster.
template<class Values>
class NetworkSensorReceiver : public SensorReceiver<Values> {
 public:
  NetworkSensorReceiver(SensorUnpackerInterface<Values> *unpacker);

 private:
  // How long to read data as fast as possible for (to clear out buffers etc).
  static const time::Time kWarmupTime;

  virtual void DoReceiveData();
  virtual void Reset();

  ReceiveSocket socket_;

  DISALLOW_COPY_AND_ASSIGN(NetworkSensorReceiver<Values>);
};

}  // namespace sensors
}  // namespace aos

#include "aos/common/sensors/sensor_receiver-tmpl.h"

#endif  // AOS_COMMON_SENSORS_SENSOR_RECEIVER_H_
