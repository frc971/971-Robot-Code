#include "bbb/sensor_reader.h"

#include <sys/types.h>
#include <unistd.h>
#include <inttypes.h>
#include <stdint.h>

#include "aos/linux_code/configuration.h"

#include "bbb/sensor_generation.q.h"
#include "bbb/crc.h"
#include "bbb/hex_byte_reader.h"

namespace bbb {
namespace {

uint32_t ReadChecksum(const ::std::string &filename) {
  HexByteReader reader(filename);
  uint32_t r = ::cape::CalculateChecksum(&reader);
  /*static const uint8_t fill[4] = {0xFF, 0xFF, 0xFF, 0xFF};
  for (size_t i = reader.GetSize(); i < 0x100000 - 0x4000; i += 4) {
    r = ::cape::CalculateChecksum(fill, 4, r);
  }*/
  return r;
}

}  // namespace

SensorReader::SensorReader(const ::std::string &cape_code)
    : hex_filename_(::std::string(::aos::configuration::GetRootDirectory()) +
                    "/main_" + cape_code + ".hex"),
      manager_(),
      packet_finder_(manager_.uart(), DATA_STRUCT_SEND_SIZE - 4),
      expected_checksum_(ReadChecksum(hex_filename_)) {
  static_assert(sizeof(SensorGeneration::reader_pid) >= sizeof(pid_t),
                "pid_t is really big?");
  ResetHappened();
}

const DataStruct *SensorReader::ReadPacket() {
  static constexpr ::aos::time::Time kResetTimeout =
      ::aos::time::Time::InSeconds(2.5);

  while (true) {
    ::aos::time::Time next_timeout = last_received_time_ + kResetTimeout;
    if (next_timeout <= ::aos::time::Time::Now()) {
      LOG(WARNING, "Too long since good packet received. Resetting.\n");
      manager_.Reset();
      ResetHappened();
    }
    if (packet_finder_.ReadPacket(next_timeout)) {
      last_received_time_ = ::aos::time::Time::Now();
      const DataStruct *data = packet_finder_.get_packet<DataStruct>();
      if (data->flash_checksum != expected_checksum_ && false) {
        LOG(WARNING, "Cape code checksum is %" PRIu32 ". Expected %" PRIu32
                     ". Reflashing.\n",
            data->flash_checksum, expected_checksum_);
        manager_.DownloadHex(hex_filename_);
        ResetHappened();
        continue;
      }
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

void SensorReader::ResetHappened() {
  sensor_generation.MakeWithBuilder().reader_pid(getpid())
      .cape_resets(cape_resets_++).Send();
  last_received_time_ = ::aos::time::Time::Now();
  last_cape_timestamp_ = 0;
}

}  // namespace bbb
