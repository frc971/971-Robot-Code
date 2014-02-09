#include "bbb/hex_byte_reader.h"

#include <string.h>
#include <errno.h>

#include "stm32flash/parsers/parser.h"
#include "stm32flash/parsers/hex.h"

#include "aos/common/logging/logging.h"

namespace bbb {
namespace {

const parser_t kParser = PARSER_HEX;

__attribute__((noreturn)) void DieParserError(parser_err_t perr) {
  if (perr == PARSER_ERR_SYSTEM) {
    LOG(ERROR, "%d: %s\n", errno, strerror(errno));
  }
  LOG(FATAL, "%s error: %s\n", kParser.name, parser_errstr(perr));
}

}  // namespace

HexByteReader::HexByteReader(const ::std::string &filename)
    : parser_status_(kParser.init()) {
  LOG(DEBUG, "reading hex file %s\n", filename.c_str());
  if (parser_status_ == NULL) {
    LOG(FATAL, "%s parser failed to initialize.\n", kParser.name);
  }
  parser_err_t perr = kParser.open(parser_status_, filename.c_str(), 0);
  if (perr != PARSER_ERR_OK) {
    DieParserError(perr);
  }
}

ssize_t HexByteReader::ReadBytes(uint8_t *dest, size_t max_bytes,
                                 const ::aos::time::Time &/*timeout*/) {
  unsigned int bytes = max_bytes;
  parser_err_t perr = kParser.read(parser_status_, dest, &bytes);
  if (perr != PARSER_ERR_OK) {
    if (perr == PARSER_ERR_SYSTEM) return -1;
    DieParserError(perr);
  }
  if (bytes == 0) return -2;
  return bytes;
}

unsigned int HexByteReader::GetSize() {
  return kParser.size(parser_status_);
}

}  // namespace bbb
