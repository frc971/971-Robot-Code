#ifndef BBB_CAPE_SRC_BBB_CAPE_MANAGER_H_
#define BBB_CAPE_SRC_BBB_CAPE_MANAGER_H_

#include <string>

#include "aos/common/macros.h"

#include "bbb/gpo.h"
#include "bbb/uart_reader.h"

namespace bbb {

// Manages the connection to the cape (including GPIOs, running the bootloader,
// setting up the serial connection, etc).
class CapeManager {
 public:
  CapeManager();

  // Downloads a .hex file using the custom bootloader.
  void DownloadHex(const ::std::string &filename);

  // Resets the cape.
  void Reset() { DoReset(false); }

  UartReader *uart() { return &uart_; }

 private:
  void DoReset(bool bootloader);

  UartReader uart_;

  Gpo reset_, custom_bootloader_;

  DISALLOW_COPY_AND_ASSIGN(CapeManager);
};

}  // namespace bbb

#endif  // BBB_CAPE_SRC_BBB_CAPE_MANAGER_H_
