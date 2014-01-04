#ifndef BBB_CAPE_SRC_BBB_SENSOR_READER_H_
#define BBB_CAPE_SRC_BBB_SENSOR_READER_H_

#include <string>

#include "aos/common/time.h"

#include "bbb/uart_reader.h"
#include "bbb/packet_finder.h"
#include "bbb/data_struct.h"

namespace bbb {

// A class that handles reading sensor values from the BBB cape.
// Automatically handles the gyro data and exposes the other data for other code
// to deal with.
class SensorReader {
 public:
  // The relative priority that tasks doing this should get run at (ie what to
  // pass to ::aos::Init(int)).
  static const int kRelativePriority = 5;

  // cape_code is the name of the code that should be deployed to the cape if
  // it's not already there.
  SensorReader(const ::std::string &cape_code);

  // Reads in 1 data packet, handles the gyro data in it, and returns a pointer
  // to it.
  const DataStruct *ReadPacket();

  // Returns the timestamp the cape sent with the most recently read packet.
  ::aos::time::Time GetCapeTimestamp();

 private:
  // Resets the cape.
  void Reset(bool reflash);
  // Called after a reset happens.
  void ResetHappened();

  UartReader reader_;
  PacketFinder packet_finder_;

  int cape_resets_ = 0;
  ::aos::time::Time last_received_time_ = ::aos::time::Time::InSeconds(0);
  uint64_t last_cape_timestamp_;
};

}  // namespace bbb

#endif  // BBB_CAPE_SRC_BBB_SENSOR_READER_H_
