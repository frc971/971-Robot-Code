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
  c2_value_ = 0;
  module_->C2 = c2_value_;
  module_->PFIFO =
      M_UART_TXFE /* Enable TX FIFO */ | M_UART_RXFE /* Enable RX FIFO */;
  module_->CFIFO =
      M_UART_TXFLUSH /* Flush TX FIFO */ | M_UART_RXFLUSH /* Flush RX FIFO */;
  c2_value_ = M_UART_TE | M_UART_RE;
  module_->C2 = c2_value_;
  // TODO(Brian): Adjust for DMA?
  module_->TWFIFO = tx_fifo_size_ - 1;
  module_->RWFIFO = 1;
}

void Uart::DoWrite(gsl::span<const char> data) {
  // In theory, we could be more efficient about this by writing the number of
  // bytes we know there's space for and only calling SpaceAvailable() (or
  // otherwise reading S1) before the final one. In practice, the FIFOs are so
  // short on this part it probably won't help anything.
  for (int i = 0; i < data.size(); ++i) {
    while (!SpaceAvailable()) {
    }
    WriteCharacter(data[i]);
  }
}

aos::SizedArray<char, 4> Uart::DoRead() {
  // In theory, we could be more efficient about this by reading the number of
  // bytes we know to be accessible and only calling DataAvailable() (or
  // otherwise reading S1) before the final one. In practice, the FIFOs are so
  // short on this part it probably won't help anything.
  aos::SizedArray<char, 4> result;
  while (DataAvailable() && !result.full()) {
    result.push_back(ReadCharacter());
  }
  return result;
}

Uart::~Uart() {
  DoDisableTransmitInterrupt();
  DoDisableReceiveInterrupt();
}

InterruptBufferedUart::~InterruptBufferedUart() {
  uart_.DisableReceiveInterrupt(DisableInterrupts());
}

void InterruptBufferedUart::Initialize(int baud_rate) {
  uart_.Initialize(baud_rate);
  {
    DisableInterrupts disable_interrupts;
    uart_.EnableReceiveInterrupt(disable_interrupts);
  }
}

void InterruptBufferedUart::Write(gsl::span<const char> data) {
  DisableInterrupts disable_interrupts;
  uart_.EnableTransmitInterrupt(disable_interrupts);
  while (!data.empty()) {
    const int bytes_written = transmit_buffer_.PushSpan(data);
    data = data.subspan(bytes_written);
    WriteCharacters(data.empty(), disable_interrupts);
    ReenableInterrupts{&disable_interrupts};
  }
}

gsl::span<char> InterruptBufferedUart::Read(gsl::span<char> buffer) {
  size_t bytes_read = 0;
  {
    DisableInterrupts disable_interrupts;
    const gsl::span<const char> read_data =
        receive_buffer_.PopSpan(buffer.size());
    std::copy(read_data.begin(), read_data.end(), buffer.begin());
    bytes_read += read_data.size();
  }
  {
    DisableInterrupts disable_interrupts;
    const gsl::span<const char> read_data =
        receive_buffer_.PopSpan(buffer.size() - bytes_read);
    std::copy(read_data.begin(), read_data.end(),
              buffer.subspan(bytes_read).begin());
    bytes_read += read_data.size();
  }
  return buffer.subspan(0, bytes_read);
}

void InterruptBufferedUart::WriteCharacters(
    bool disable_empty, const DisableInterrupts &disable_interrupts) {
  while (true) {
    if (transmit_buffer_.empty()) {
      if (disable_empty) {
        uart_.DisableTransmitInterrupt(disable_interrupts);
      }
      return;
    }
    if (!uart_.SpaceAvailable()) {
      return;
    }
    uart_.WriteCharacter(transmit_buffer_.PopSingle());
  }
}

void InterruptBufferedUart::ReadCharacters(const DisableInterrupts &) {
  while (true) {
    if (receive_buffer_.full()) {
      return;
    }
    if (!uart_.DataAvailable()) {
      return;
    }
    receive_buffer_.PushSingle(uart_.ReadCharacter());
  }
}

}  // namespace teensy
}  // namespace frc971
