#include "bbb/cape_flasher.h"

#include <string.h>
#include <inttypes.h>

#include "aos/common/logging/logging.h"

#include "bbb/crc.h"
#include "bbb/byte_io.h"

namespace bbb {
namespace {
namespace command {

const uint8_t kAck = 0x79;
const uint8_t kNack = 0x1F;

}  // namespace command
}  // namespace

CapeFlasher::CapeFlasher(ByteReaderWriterInterface *uart) : uart_(uart) {}

void CapeFlasher::Download(ByteReaderInterface *file) {
  uint8_t bootloader_receive = 0;
  uint8_t buffer[256];
  ssize_t bootloader_read = 0;

  while (bootloader_read <= 0 || bootloader_receive != command::kNack) {
    bootloader_read = uart_->ReadBytes(&bootloader_receive, 1,
                                       ::aos::time::Time::InSeconds(2));
    if (bootloader_read == -1) {
      PLOG(WARNING, "reading from %p (uart) failed", uart_);
    } else if (bootloader_read == -2) {
      LOG(WARNING, "timeout reading from uart %p\n", uart_);
    }
  }

  while (true) {
    size_t total_read = 0;
    bool done = false;
    while (total_read < sizeof(buffer)) {
      ssize_t read = file->ReadBytes(buffer, sizeof(buffer));
      if (read == -2) {
        if (total_read == 0) return;
        done = true;
        memset(buffer + total_read, 0xFF, sizeof(buffer) - total_read);
        total_read = sizeof(buffer);
      } else if (read == -1) {
        PLOG(FATAL, "reading from file %p failed", file);
      } else {
        total_read += read;
      }
    }

    WriteBuffer(buffer, sizeof(buffer));
    if (done) return;
  }
}

void CapeFlasher::WriteBuffer(uint8_t *buffer, size_t size) {
  while (true) {
    uart_->WriteBytes(buffer, size);
    uint32_t checksum = cape::CalculateChecksum(buffer, size);
    uart_->WriteBytes(reinterpret_cast<uint8_t *>(&checksum),
                      sizeof(checksum));

    uint8_t bootloader_receive;
    ssize_t bootloader_read = uart_->ReadBytes(
        &bootloader_receive, 1, ::aos::time::Time::InSeconds(2));
    if (bootloader_read == -1) {
      PLOG(WARNING, "reading from %p (uart) failed", uart_);
    } else if (bootloader_read == -2) {
      LOG(WARNING, "timeout reading from uart %p\n", uart_);
      do {
        uint8_t byte = 0;
        uart_->WriteBytes(&byte, 1);
        bootloader_read = uart_->ReadBytes(&bootloader_receive, 1,
                                           ::aos::time::Time::InSeconds(0.1));
      } while (bootloader_read <= 0 || bootloader_receive != command::kNack);
      switch (bootloader_receive) {
        case command::kAck:
          return;
        case command::kNack:
          break;
        default:
          LOG(WARNING, "unknown bootloader response %" PRIu8 "\n",
              bootloader_receive);
      }
    }
  }
}

}  // namespace bbb
