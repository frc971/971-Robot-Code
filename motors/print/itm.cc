#include "motors/print/itm.h"

#include "motors/core/itm.h"

namespace frc971 {
namespace motors {
namespace {

template<int kPort> void WriteToPort(gsl::span<const char> buffer) {
  // This ignores memory barriers etc, because it will be called by
  // CreatePrinting which must be called before any interrupts are enabled. That
  // means the only thing we need to worry about is actually getting it
  // initialized with a minimal number of cycles.
  static bool is_initialized = false;
  if (__builtin_expect(!is_initialized, false)) {
    is_initialized = true;
    itm::Initialize();
  }

  const char *next_address = buffer.data();
  int remaining_bytes = buffer.size();

  // Write small chunks to make the address even.
  if (remaining_bytes >= 1 && (reinterpret_cast<uintptr_t>(next_address) & 1)) {
    uint8_t value;
    memcpy(&value, next_address, 1);
    itm::Write8(kPort, value);
    next_address += 1;
    remaining_bytes -= 1;
  }
  if (remaining_bytes >= 2 && (reinterpret_cast<uintptr_t>(next_address) & 2)) {
    uint16_t value;
    memcpy(&value, next_address, 2);
    itm::Write16(kPort, value);
    next_address += 2;
    remaining_bytes -= 2;
  }

  // Write big chunks while we can.
  while (remaining_bytes >= 4) {
    uint32_t value;
    memcpy(&value, next_address, 4);
    itm::Write32(kPort, value);
    next_address += 4;
    remaining_bytes -= 4;
  }

  // Write out any remaining uneven bytes on the end.
  if (remaining_bytes >= 2) {
    uint16_t value;
    memcpy(&value, next_address, 2);
    itm::Write16(kPort, value);
    next_address += 2;
    remaining_bytes -= 2;
  }
  if (remaining_bytes >= 1) {
    uint8_t value;
    memcpy(&value, next_address, 1);
    itm::Write8(kPort, value);
    next_address += 1;
    remaining_bytes -= 1;
  }
}

}  // namespace

::std::unique_ptr<PrintingImplementation> CreatePrinting(
    const PrintingParameters & /*parameters*/) {
  return ::std::unique_ptr<PrintingImplementation>(new ItmPrinting());
}

extern "C" int _write(const int /*file*/, char *const ptr, const int len) {
  WriteToPort<0>(gsl::span<const char>(ptr, len));
  return len;
}

ItmPrinting::ItmPrinting() {
  // Make sure we run the one-time initialization. It's important to do it here
  // to ensure it's complete before interrupts are enabled, because it's not
  // interrupt-safe.
  _write(0, nullptr, 0);
}

int ItmPrinting::WriteStdout(gsl::span<const char> buffer) {
  WriteToPort<0>(buffer);
  return buffer.size();
}

int ItmPrinting::WriteDebug(gsl::span<const char> buffer) {
  WriteToPort<1>(buffer);
  return buffer.size();
}

}  // namespace motors
}  // namespace frc971
