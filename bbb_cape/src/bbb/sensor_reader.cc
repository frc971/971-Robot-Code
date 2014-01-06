#include "bbb/sensor_reader.h"

#include <sys/types.h>
#include <unistd.h>
#include <inttypes.h>

#include "bbb/sensor_generation.q.h"

namespace bbb {

SensorReader::SensorReader(const ::std::string &/*cape_code*/)
    : reader_(750000), packet_finder_(&reader_, DATA_STRUCT_SEND_SIZE - 4) {
  static_assert(sizeof(SensorGeneration::reader_pid) >= sizeof(pid_t),
                "pid_t is really big");
  ResetHappened();
}

const DataStruct *SensorReader::ReadPacket() {
  static constexpr ::aos::time::Time kTimeout =
      ::aos::time::Time::InSeconds(0.5);

  while (true) {
    ::aos::time::Time next_timeout = last_received_time_ + kTimeout;
    if (next_timeout <= ::aos::time::Time::Now()) {
      LOG(WARNING, "too long since good packet received\n");
      Reset(false);
    }
    if (packet_finder_.ReadPacket(next_timeout)) {
      last_received_time_ = ::aos::time::Time::Now();
      const DataStruct *data = packet_finder_.get_packet<DataStruct>();
      // TODO(brians): Check the flash checksum and reflash it if necessary.
      if (data->timestamp < last_cape_timestamp_) {
        LOG(WARNING, "cape timestamp decreased: %" PRIu64 " to %" PRIu64 "\n",
            last_cape_timestamp_, data->timestamp);
        ResetHappened();
      }
      last_cape_timestamp_ = data->timestamp;
      return data;
    }
    LOG(WARNING, "receiving packet failed\n");
  }
}

::aos::time::Time SensorReader::GetCapeTimestamp() {
  return ::aos::time::Time::InUS(last_cape_timestamp_ * 10);
}

void SensorReader::Reset(bool reflash) {
  LOG(INFO, "Reset(%s)\n", reflash ? "true" : "false");
  // TODO(brians): Actually reset it and maybe reflash it.
  ResetHappened();
}

void SensorReader::ResetHappened() {
  sensor_generation.MakeWithBuilder().reader_pid(getpid())
      .cape_resets(cape_resets_++).Send();
  last_received_time_ = ::aos::time::Time::Now();
  last_cape_timestamp_ = 0;
}

}  // namespace bbb
