// Copyright 2012 Google Inc. All Rights Reserved.
//
// Modified by FRC Team 971.
//

#include "ghexdump.h"

#include <sstream>
#include <iomanip>
#include <stddef.h>
#include <stdint.h>
#include <cstring>

namespace glibusb {

std::string Dump(const void *buf, size_t n) {
  const int kBytesPerRow = 16;
  const uint8_t *p = static_cast<const uint8_t *>(CHECK_NOTNULL(buf));
  std::stringstream dump;
  for (uint32_t i = 0; i < n;) {
    dump << "0x" 
	 << std::hex << std::setw(8) << std::setfill('0') << i
	 << ": ";
    for (int j = 0; j < kBytesPerRow; ++j) {
      auto o = i + j;
      if (o < n) {
	auto b = p[o];
	dump << std::hex << std::setw(2) << std::setfill('0') << int(b) << ' ';
      } else {
	dump << "   ";
      }
    }
    dump << " | ";
    for (int j = 0; j < kBytesPerRow; ++j) {
      auto o = i + j;
      if (o == n) {
	break;
      }
      auto b = p[o];
      if (b > ' ' && b <= '~') {
	dump << char(b);
      } else {
	dump << '.';
      }
    }
    dump << std::endl;
    i += kBytesPerRow;
  }
  return dump.str();
}

}  // namespace glibusb

