#include "bbb/sensor_reader.h"

#include <sys/types.h>
#include <unistd.h>
#include <inttypes.h>
#include <stdint.h>

#include "stm32flash/parsers/parser.h"
#include "stm32flash/parsers/hex.h"

#include "aos/linux_code/configuration.h"

#include "bbb/sensor_generation.q.h"
#include "bbb/crc.h"

namespace bbb {
namespace {

__attribute__((noreturn)) void PrintParserError(parser_t *parser,
                                                parser_err_t perr) {
  if (perr == PARSER_ERR_SYSTEM) {
    LOG(ERROR, "%d: %s\n", errno, strerror(errno));
  }
  LOG(FATAL, "%s error: %s\n", parser->name, parser_errstr(perr));
}

uint32_t ReadChecksum(const ::std::string &cape_code) {
  parser_t *parser = &PARSER_HEX;
  void *p_st = parser->init();
  if (p_st == NULL) {
    LOG(FATAL, "%s parser failed to initialize.\n", parser->name);
  }
  ::std::string filename =
      ::std::string(::aos::configuration::GetRootDirectory()) + "/main_" +
      cape_code + ".hex";
  parser_err_t perr = parser->open(p_st, filename.c_str(), 0);
  if (perr != PARSER_ERR_OK) {
    PrintParserError(parser, perr);
  }

  const unsigned int allocated_size = parser->size(p_st);
  uint8_t *buffer = new uint8_t[allocated_size];
  unsigned int read_size = allocated_size;
  if (parser->read(p_st, buffer, &read_size) != PARSER_ERR_OK) {
    PrintParserError(parser, perr);
  }
  if (allocated_size != read_size) {
    LOG(WARNING, "expected %u bytes but got %u\n", allocated_size, read_size);
  }

  uint32_t r = ::cape::CalculateChecksum(buffer, read_size);
  parser->close(p_st);
  return r;
}

}  // namespace

SensorReader::SensorReader(const ::std::string &cape_code)
    : reader_(750000),
      packet_finder_(&reader_, DATA_STRUCT_SEND_SIZE - 4),
      expected_checksum_(ReadChecksum(cape_code)) {
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
