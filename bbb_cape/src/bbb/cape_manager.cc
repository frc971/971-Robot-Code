#include "bbb/cape_manager.h"

#include <string.h>
#include <errno.h>

#include "aos/common/time.h"

#include "bbb/cape_flasher.h"
#include "bbb/hex_byte_reader.h"

namespace bbb {

CapeManager::CapeManager()
    : uart_(750000), reset_(2, 5, true), custom_bootloader_(2, 3, false) {}

void CapeManager::DownloadHex(const ::std::string &filename) {
  HexByteReader file(filename);
  CapeFlasher flasher(&uart_);
  DoReset(true);
  flasher.Download(&file);
  DoReset(false);
}

void CapeManager::DoReset(bool bootloader) {
  static constexpr ::aos::time::Time kWaitTime =
      ::aos::time::Time::InSeconds(0.1);
  custom_bootloader_.Set(!bootloader);
  reset_.Set(false);
  ::aos::time::SleepFor(kWaitTime);
  reset_.Set(true);
  ::aos::time::SleepFor(kWaitTime);
}

}  // namespace bbb
