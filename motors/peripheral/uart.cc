#include "motors/peripheral/uart.h"

#include <stdint.h>

namespace frc971 {
namespace teensy {

// Currently hard-coded for 8-bit + no parity + start bit + stop bit.
void Uart::Initialize(int baud_rate) {
  {
    // UART baud rate = UART module clock / (16 * (SBR[12:0] + BRFD))
    // BRFD = BRFA (bitfield) / 32
    const int desired_receiver_clock = baud_rate * 16;
    const int sbr_and_brfd32 =
        ((static_cast<int64_t>(module_clock_frequency_) * UINT64_C(64) /
          static_cast<int64_t>(desired_receiver_clock)) +
         1) /
        2;
    const int sbr = sbr_and_brfd32 / 32;
    const int brfa = sbr_and_brfd32 % 32;

    module_->BDH = (sbr >> 8) & 0x1F;
    module_->BDL = sbr & 0xFF;
    module_->C1 = M_UART_ILT /* only detect idle after stop bit */ |
                  M_UART_PT /* odd parity */;
    module_->C4 = V_UART_BRFA(brfa);
  }
  {
    const uint8_t pfifo = module_->PFIFO;
    tx_fifo_size_ = G_UART_TXFIFOSIZE(pfifo);
    rx_fifo_size_ = G_UART_RXFIFOSIZE(pfifo);
  }

  // When C1[M] is set and C4[M10] is cleared, the UART is configured for 9-bit
  // data characters. If C1[PE] is enabled, the ninth bit is either C3[T8/R8] or
  // the internally generated parity bit

  // TODO(Brian): M_UART_TIE /* Enable TX interrupt or DMA */ |
  // M_UART_RIE /* Enable RX interrupt or DMA */
  // Also set in C5: M_UART_TDMAS /* Do DMA for TX */ |
  // M_UART_RDMAS /* Do DMA for RX */
  c2_value_ = M_UART_TE;
  module_->C2 = c2_value_;
  module_->PFIFO =
      M_UART_TXFE /* Enable TX FIFO */ | M_UART_RXFE /* Enable RX FIFO */;
  module_->CFIFO =
      M_UART_TXFLUSH /* Flush TX FIFO */ | M_UART_RXFLUSH /* Flush RX FIFO */;
  // TODO(Brian): Adjust for DMA?
  module_->TWFIFO = tx_fifo_size_ - 1;
  module_->RWFIFO = rx_fifo_size_ - 1;
}

void Uart::DoWrite(gsl::span<const char> data) {
  for (int i = 0; i < data.size(); ++i) {
    while (!SpaceAvailable()) {
    }
    WriteCharacter(data[i]);
  }
}

Uart::~Uart() { DisableTransmitInterrupt(); }

void InterruptBufferedUart::Initialize(int baud_rate) {
  uart_.Initialize(baud_rate);
}

void InterruptBufferedUart::Write(gsl::span<const char> data) {
  DisableInterrupts disable_interrupts;
  uart_.EnableTransmitInterrupt();
  static_assert(buffer_.size() >= 8,
                "Need enough space to not turn the interrupt off each time");
  while (!data.empty()) {
    const int bytes_written = buffer_.PushSpan(data);
    data = data.subspan(bytes_written);
    WriteCharacters(data.empty(), disable_interrupts);
    ReenableInterrupts{&disable_interrupts};
  }
}

void InterruptBufferedUart::WriteCharacters(bool disable_empty,
                                            const DisableInterrupts &) {
  while (true) {
    if (buffer_.empty()) {
      if (disable_empty) {
        uart_.DisableTransmitInterrupt();
      }
      return;
    }
    if (!uart_.SpaceAvailable()) {
      return;
    }
    uart_.WriteCharacter(buffer_.PopSingle());
  }
}

}  // namespace teensy
}  // namespace frc971
