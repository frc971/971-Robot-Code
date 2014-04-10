#include "bbb/sensor_reader.h"

#include <sys/types.h>
#include <unistd.h>
#include <inttypes.h>
#include <stdint.h>

#include "aos/linux_code/configuration.h"
#include "aos/common/controls/sensor_generation.q.h"

#include "bbb/crc.h"
#include "bbb/hex_byte_reader.h"

namespace bbb {
namespace {

uint32_t ReadChecksum(const ::std::string &filename) {
  HexByteReader reader(filename);
  {
    static const size_t kSkipBytes = 0x4000;
    size_t i = 0;
    uint8_t buffer[1024];
    while (i < kSkipBytes) {
      ssize_t read = reader.ReadBytes(
          buffer, ::std::min<size_t>(sizeof(buffer), kSkipBytes - i));
      if (read < 0) {
        LOG(FATAL, "error skipping bytes before actual data\n");
      }
      i += read;
    }
  }
  return ::cape::CalculateChecksum(&reader);
}

}  // namespace

SensorReader::SensorReader(const ::std::string &cape_code)
    : hex_filename_(::std::string(::aos::configuration::GetRootDirectory()) +
                    "/main_" + cape_code + ".hex"),
      manager_(),
      packet_finder_(manager_.uart(), DATA_STRUCT_SEND_SIZE - 4),
      expected_checksum_(ReadChecksum(hex_filename_)) {
  static_assert(
      sizeof(::aos::controls::SensorGeneration::reader_pid) >= sizeof(pid_t),
      "pid_t is really big?");
  ResetHappened();
}

const DataStruct *SensorReader::ReadPacket() {
  static constexpr ::aos::time::Time kResetTimeout =
      ::aos::time::Time::InSeconds(5);

  while (true) {
    receive_failed_.Print();

    ::aos::time::Time next_timeout = last_received_time_ + kResetTimeout;
    if (next_timeout <= ::aos::time::Time::Now()) {
      LOG(WARNING, "Too long since good packet received. Resetting.\n");
      manager_.Reset();
      ResetHappened();
      continue;
    }
    if (packet_finder_.ReadPacket(next_timeout)) {
      last_received_time_ = ::aos::time::Time::Now();
      const DataStruct *data = packet_finder_.get_packet<DataStruct>();
      if (data->flash_checksum != expected_checksum_) {
        // TODO(brians): Fix the custom bootloader stuff and then change this to
        // WARNING.
        LOG(FATAL, "Cape code checksum is 0x%" PRIx32 ". Expected 0x%" PRIx32
                   ". Reflashing.\n",
            data->flash_checksum, expected_checksum_);
        manager_.DownloadHex(hex_filename_);
        ResetHappened();
        continue;
      }
      if (data->timestamp < last_cape_timestamp_) {
        // TODO(brians): A common failure mode of the cape is to send the same 2
        // packets (from the ring buffer) out repeatedly. Detect that and reset
        // it.
        LOG(WARNING, "cape timestamp decreased: %" PRIu64 " to %" PRIu64 "\n",
            last_cape_timestamp_, data->timestamp);
        ResetHappened();
        continue;
      }
      last_cape_timestamp_ = data->timestamp;
      return data;
    }
    LOG_INTERVAL(receive_failed_);
  }
}

::aos::time::Time SensorReader::GetCapeTimestamp() {
  return ::aos::time::Time::InUS(last_cape_timestamp_ * 10);
}

void SensorReader::ResetHappened() {
  ::aos::controls::sensor_generation.MakeWithBuilder().reader_pid(getpid())
      .cape_resets(cape_resets_++).Send();
  last_received_time_ = ::aos::time::Time::Now();
  last_cape_timestamp_ = 0;
}

}  // namespace bbb
