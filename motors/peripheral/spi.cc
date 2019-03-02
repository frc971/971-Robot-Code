#include "motors/peripheral/spi.h"

#include <stdint.h>

#include "motors/core/time.h"

namespace frc971 {
namespace teensy {

Spi::~Spi() {
  DisableTransmitInterrupt();
  DisableReceiveInterrupt();
  FlushInterruptRequests();
}

void Spi::Initialize() {
  // Use all the defaults (including slave mode).
  mcr_value_ =
      V_SPI_PCSIS(1) /* CS0 is active low. It appears to ignore this though? */;
  module_->MCR = mcr_value_;
  module_->CTAR[0] = V_SPI_FMSZ(7) /* 8 bit frames */ |
                     M_SPI_CPOL /* Idle high clock */ |
                     M_SPI_CPHA /* Data changed on leading clock edge */;
}

void Spi::ClearQueues() {
  module_->MCR = mcr_value_ | M_SPI_HALT;
  const bool receive_overflow = module_->SR & M_SPI_RFOF;
  module_->MCR = mcr_value_ | M_SPI_CLR_TXF | M_SPI_CLR_RXF | M_SPI_DIS_TXF |
                 M_SPI_DIS_RXF | M_SPI_HALT;
  if (receive_overflow) {
    (void)module_->POPR;
  }
  module_->SR = M_SPI_TFUF | M_SPI_TFFF | M_SPI_RFOF | M_SPI_RFDF;
  module_->MCR = mcr_value_;
}

InterruptBufferedSpi::~InterruptBufferedSpi() {
  spi_.DisableReceiveInterrupt();
  spi_.FlushInterruptRequests();
}

void InterruptBufferedSpi::Initialize() {
  spi_.Initialize();
  transmit_buffer_.clear();
  receive_buffer_.clear();
  frames_to_receive_ = 0;
}

void InterruptBufferedSpi::ClearQueues(const DisableInterrupts &) {
  spi_.ClearQueues();
  transmit_buffer_.clear();
  receive_buffer_.clear();
  frames_to_receive_ = 0;
  spi_.DisableTransmitInterrupt();
  spi_.DisableReceiveInterrupt();
  spi_.FlushInterruptRequests();
}

void InterruptBufferedSpi::Write(gsl::span<const char> data,
                                 DisableInterrupts *disable_interrupts) {
  frames_to_receive_ += data.size();
  // Until we get all of the data queued, we'll call WriteFrames from our
  // context, so don't enable the interrupt yet.
  while (!data.empty()) {
    const int bytes_written = transmit_buffer_.PushSpan(data);
    data = data.subspan(bytes_written);
    while (ReadAndWriteFrame(data.empty(), *disable_interrupts)) {
    }
    ReenableInterrupts{disable_interrupts};
  }
  // If there's still data queued, then we need to enable the interrupt so it
  // can push it to the hardware.
  if (!transmit_buffer_.empty()) {
    spi_.EnableTransmitInterrupt();
  }
  if (frames_to_receive_ > 0) {
    spi_.EnableReceiveInterrupt();
  }
  if (!transmit_buffer_.empty() || frames_to_receive_ > 0) {
    spi_.FlushInterruptRequests();
  }
}

gsl::span<char> InterruptBufferedSpi::Read(
    gsl::span<char> buffer, DisableInterrupts *disable_interrupts) {
  size_t bytes_read = 0;
  {
    const gsl::span<const char> read_data =
        receive_buffer_.PopSpan(buffer.size());
    std::copy(read_data.begin(), read_data.end(), buffer.begin());
    bytes_read += read_data.size();
  }

  ReenableInterrupts{disable_interrupts};

  {
    const gsl::span<const char> read_data =
        receive_buffer_.PopSpan(buffer.size() - bytes_read);
    std::copy(read_data.begin(), read_data.end(),
              buffer.subspan(bytes_read).begin());
    bytes_read += read_data.size();
  }

  return buffer.first(bytes_read);
}

bool InterruptBufferedSpi::WriteFrame(bool disable_empty,
                                      const DisableInterrupts &) {
  if (transmit_buffer_.empty()) {
    if (disable_empty) {
      spi_.DisableTransmitInterrupt();
    }
    return false;
  }
  if (!spi_.SpaceAvailable()) {
    return false;
  }
  spi_.WriteFrame(transmit_buffer_.PopSingle());
  return true;
}

bool InterruptBufferedSpi::ReadFrame(const DisableInterrupts &) {
  if (!spi_.DataAvailable()) {
    return false;
  }
  const auto frame = spi_.ReadFrame();
  --frames_to_receive_;
  if (frames_to_receive_ <= 0) {
    spi_.DisableReceiveInterrupt();
  }
  if (receive_buffer_.full()) {
    return false;
  }
  receive_buffer_.PushSingle(frame);
  return true;
}

}  // namespace teensy
}  // namespace frc971
