#include "aos/time/time.h"
#include "motors/core/kinetis.h"
#include "motors/core/time.h"
#include "motors/peripheral/configuration.h"
#include "motors/peripheral/spi.h"
#include "motors/peripheral/uart.h"
#include "motors/print/print.h"
#include "motors/util.h"
#include "third_party/GSL/include/gsl/gsl"
#include "y2019/jevois/cobs.h"
#include "y2019/jevois/spi.h"
#include "y2019/jevois/uart.h"

using frc971::teensy::InterruptBufferedUart;
using frc971::teensy::InterruptBufferedSpi;

// All indices here refer to the ports as numbered on the PCB.

namespace frc971 {
namespace jevois {
namespace {

// Holds all of our hardware UARTs. There is exactly one global instance for
// interrupt handlers to access.
struct Uarts {
  Uarts() {
    DisableInterrupts disable_interrupts;
    global_instance = this;
  }
  ~Uarts() {
    DisableInterrupts disable_interrupts;
    global_instance = nullptr;
  }
  Uarts(const Uarts &) = delete;
  Uarts &operator=(const Uarts &) = delete;

  void Initialize(int baud_rate) {
    cam0.Initialize(baud_rate);
    cam1.Initialize(baud_rate);
    cam2.Initialize(baud_rate);
    cam3.Initialize(baud_rate);
    cam4.Initialize(baud_rate);
  }

  InterruptBufferedUart cam0{&UART1, F_CPU};
  InterruptBufferedUart cam1{&UART0, F_CPU};
  InterruptBufferedUart cam2{&UART2, BUS_CLOCK_FREQUENCY};
  InterruptBufferedUart cam3{&UART3, BUS_CLOCK_FREQUENCY};
  InterruptBufferedUart cam4{&UART4, BUS_CLOCK_FREQUENCY};

  static Uarts *global_instance;
};

Uarts *Uarts::global_instance = nullptr;

// Manages the transmit buffer to a single camera.
//
// We have to add delays between sending each byte in order for the camera to
// successfully receive them.
struct TransmitBuffer {
  TransmitBuffer(InterruptBufferedUart *camera_in) : camera(camera_in) {}
  InterruptBufferedUart *const camera;

  frc971::teensy::UartBuffer<1024> buffer;
  aos::monotonic_clock::time_point last_send = aos::monotonic_clock::min_time;

  // Sends a byte to the camera if it's time.
  void Tick(aos::monotonic_clock::time_point now) {
    if (buffer.empty()) {
      return;
    }
    if (now < last_send + std::chrono::milliseconds(1)) {
      return;
    }
    last_send = now;
    camera->Write(std::array<char, 1>{{buffer.PopSingle()}});
  }

  // Queues up another packet to send, only if the previous one has finished.
  void MaybeWritePacket(const CameraCalibration &calibration) {
    if (!buffer.empty()) {
      return;
    }
    const auto serialized = UartPackToCamera(calibration);
    buffer.PushSingle(0);
    if (buffer.PushSpan(serialized) == static_cast<int>(serialized.size())) {
      buffer.PushSingle(0);
    }
  }
};

InterruptBufferedSpi *global_spi_instance = nullptr;

// Manages queueing a transfer to send via SPI.
class SpiQueue {
 public:
  SpiQueue() {
    DisableInterrupts disable_interrupts;
    global_instance = this;
  }
  ~SpiQueue() {
    DisableInterrupts disable_interrupts;
    global_instance = nullptr;
  }
  SpiQueue(const SpiQueue &) = delete;
  SpiQueue &operator=(const SpiQueue &) = delete;

  tl::optional<gsl::span<const char, spi_transfer_size()>> Tick() {
    {
      DisableInterrupts disable_interrupts;
      if (waiting_for_enable_ || waiting_for_disable_) {
        return tl::nullopt;
      }
    }
    const auto now = aos::monotonic_clock::now();
    if (TransferTimedOut(now)) {
      printf("SPI timeout with %d left\n", static_cast<int>(to_receive_.size()));
      WaitForNextTransfer();
      return tl::nullopt;
    }
    {
      DisableInterrupts disable_interrupts;
      if (!PERIPHERAL_BITBAND(GPIOA_PDIR, 17) &&
          cs_deassert_time_ == aos::monotonic_clock::max_time) {
        cs_deassert_time_ = now;
      }
    }
    if (DeassertHappened(now)) {
      printf("CS deasserted with %d left\n", static_cast<int>(to_receive_.size()));
      WaitForNextTransfer();
      return tl::nullopt;
    }
    bool all_done;
    {
      DisableInterrupts disable_interrupts;
      if (received_dummy_) {
        to_receive_ = to_receive_.subspan(
            global_spi_instance->Read(to_receive_, &disable_interrupts).size());
        all_done = to_receive_.empty();
      } else {
        std::array<char, 1> dummy_data;
        if (global_spi_instance->Read(dummy_data, &disable_interrupts).size() >=
            1) {
          received_dummy_ = true;
        }
        all_done = false;
      }
    }
    if (all_done) {
      WaitForNextTransfer();
      return received_transfer_;
    }
    return tl::nullopt;
  }

  void HandleInterrupt() {
    DisableInterrupts disable_interrupts;
    if (waiting_for_disable_) {
      if (!PERIPHERAL_BITBAND(GPIOA_PDIR, 17)) {
        PORTA_PCR17 =
            PORT_PCR_MUX(1) | PORT_PCR_IRQC(0xC) /* Interrupt when logic 1 */;
        // Clear the interrupt flag now that we've reconfigured it.
        PORTA_ISFR = 1 << 17;
        waiting_for_disable_ = false;
      } else {
        // Clear the interrupt flag. It shouldn't trigger again immediately
        // because the pin is still asserted.
        PORTA_ISFR = 1 << 17;
      }
      return;
    }
    if (waiting_for_enable_) {
      if (PERIPHERAL_BITBAND(GPIOA_PDIR, 17)) {
        global_spi_instance->ClearQueues(disable_interrupts);
        // Tell the SPI peripheral its CS is asserted.
        PERIPHERAL_BITBAND(GPIOB_PDOR, 17) = 0;
        // Disable interrupts on the enable pin. We'll re-enable once we finish
        // the transfer.
        PORTA_PCR17 = PORT_PCR_MUX(1);
        // Clear the interrupt flag now that we've reconfigured it.
        PORTA_ISFR = 1 << 17;
        if (have_transfer_) {
          global_spi_instance->Write(transfer_, &disable_interrupts);
          have_transfer_ = false;
        } else {
          printf("Writing dummy SPI frame\n");
          // If we don't have anything, just write 0s to avoid getting the
          // hardware confused.
          global_spi_instance->Write(SpiTransfer{}, &disable_interrupts);
        }
        // Queue up a dummy byte at the end. This won't actually be sent,
        // because the first byte we do send will be garbage, but it will
        // synchronize our queues so we receive all the useful data bytes.
        global_spi_instance->Write(std::array<char, 1>(), &disable_interrupts);
        waiting_for_enable_ = false;
        receive_start_ = aos::monotonic_clock::now();
        cs_deassert_time_ = aos::monotonic_clock::max_time;
        // To make debugging easier.
        received_transfer_.fill(0);
      } else {
        // Clear the interrupt flag. It shouldn't trigger again immediately
        // because the pin is still asserted.
        PORTA_ISFR = 1 << 17;
      }
      return;
    }
    // We shouldn't ever get here. Clear all the flags and hope they don't get
    // re-asserted immediately.
    PORTA_ISFR = UINT32_C(0xFFFFFFFF);
  }

  void UpdateTransfer(const SpiTransfer &transfer, const DisableInterrupts &) {
    have_transfer_ = true;
    transfer_ = transfer;
  }

  // Returns whether a transfer is currently queued. This will be true between a
  // call to UpdateTransfer and that transfer actually being moved out to the
  // hardware.
  bool HaveTransfer(const DisableInterrupts &) const { return have_transfer_; }

  static SpiQueue *global_instance;

 private:
  void WaitForNextTransfer() {
    to_receive_ = received_transfer_;
    received_dummy_ = false;
    {
      DisableInterrupts disable_interrupts;
      waiting_for_enable_ = true;
      waiting_for_disable_ = true;
      PORTA_PCR17 =
          PORT_PCR_MUX(1) | PORT_PCR_IRQC(0x8) /* Interrupt when logic 0 */;
      // Clear the interrupt flag now that we've reconfigured it.
      PORTA_ISFR = 1 << 17;
    }
    // Tell the SPI peripheral its CS is de-asserted.
    PERIPHERAL_BITBAND(GPIOB_PDOR, 17) = 1;
  }

  bool TransferTimedOut(aos::monotonic_clock::time_point now) {
    DisableInterrupts disable_interrupts;
    // TODO: Revise this timeout.
    return now - std::chrono::milliseconds(50) > receive_start_;
  }

  bool DeassertHappened(aos::monotonic_clock::time_point now) {
    DisableInterrupts disable_interrupts;
    return now - std::chrono::microseconds(50) > cs_deassert_time_;
  }

  bool waiting_for_enable_ = true;
  bool waiting_for_disable_ = false;
  bool have_transfer_ = false;
  SpiTransfer transfer_;
  bool received_dummy_ = false;
  SpiTransfer received_transfer_;
  gsl::span<char> to_receive_ = received_transfer_;
  aos::monotonic_clock::time_point receive_start_;
  aos::monotonic_clock::time_point cs_deassert_time_;
};

SpiQueue *SpiQueue::global_instance = nullptr;

// All methods here must be fully synchronized by the caller.
class FrameQueue {
 public:
  FrameQueue() = default;
  FrameQueue(const FrameQueue &) = delete;
  FrameQueue &operator=(const FrameQueue &) = delete;

  void UpdateFrame(int camera, const Frame &frame) {
    frames_[camera].targets = frame.targets;
    frames_[camera].capture_time = aos::monotonic_clock::now() - frame.age;
    const aos::SizedArray<int, 3> old_last_frames = last_frames_;
    last_frames_.clear();
    for (int index : old_last_frames) {
      if (index != camera) {
        last_frames_.push_back(index);
      }
    }
  }

  // Creates and returns a transfer with all the current information.
  //
  // This does not actually record these frames as transferred until
  // RemoveLatestFrames() is called.
  SpiTransfer MakeTransfer();

  // Records the frames represented in the result of the latest MakeTransfer()
  // call as being transferred, so they will not be represented in subsequent
  // MakeTransfer() calls.
  void RemoveLatestFrames() {
    for (int index : last_frames_) {
      frames_[index].capture_time = aos::monotonic_clock::min_time;
    }
    last_frames_.clear();
  }

 private:
  struct FrameData {
    aos::SizedArray<Target, 3> targets;
    aos::monotonic_clock::time_point capture_time =
        aos::monotonic_clock::min_time;
  };

  std::array<FrameData, 5> frames_;
  // The indices into frames_ which we returned in the last MakeTransfer() call.
  aos::SizedArray<int, 3> last_frames_;
};

SpiTransfer FrameQueue::MakeTransfer() {
  aos::SizedArray<int, 5> oldest_indices;
  for (size_t i = 0; i < frames_.size(); ++i) {
    if (frames_[i].capture_time != aos::monotonic_clock::min_time) {
      oldest_indices.push_back(i);
    }
  }
  std::sort(oldest_indices.begin(), oldest_indices.end(), [this](int a, int b) {
    return frames_[a].capture_time < frames_[b].capture_time;
  });

  TeensyToRoborio message;
  last_frames_.clear();
  for (int i = 0; i < std::min<int>(oldest_indices.size(), 3); ++i) {
    const int index = oldest_indices[i];
    const FrameData &frame = frames_[index];
    const auto age = aos::monotonic_clock::now() - frame.capture_time;
    const auto rounded_age = aos::time::round<camera_duration>(age);
    message.frames.push_back({frame.targets, rounded_age});
    last_frames_.push_back(index);
  }
  return SpiPackToRoborio(message);
}

extern "C" {

void *__stack_chk_guard = (void *)0x67111971;
void __stack_chk_fail(void) {
  while (true) {
    GPIOC_PSOR = (1 << 5);
    printf("Stack corruption detected\n");
    delay(1000);
    GPIOC_PCOR = (1 << 5);
    delay(1000);
  }
}

extern char *__brkval;
extern uint32_t __bss_ram_start__[];
extern uint32_t __heap_start__[];
extern uint32_t __stack_end__[];

void uart0_status_isr(void) {
  DisableInterrupts disable_interrupts;
  Uarts::global_instance->cam1.HandleInterrupt(disable_interrupts);
}

void uart1_status_isr(void) {
  DisableInterrupts disable_interrupts;
  Uarts::global_instance->cam0.HandleInterrupt(disable_interrupts);
}

void uart2_status_isr(void) {
  DisableInterrupts disable_interrupts;
  Uarts::global_instance->cam2.HandleInterrupt(disable_interrupts);
}

void uart3_status_isr(void) {
  DisableInterrupts disable_interrupts;
  Uarts::global_instance->cam3.HandleInterrupt(disable_interrupts);
}

void uart4_status_isr(void) {
  DisableInterrupts disable_interrupts;
  Uarts::global_instance->cam4.HandleInterrupt(disable_interrupts);
}

void spi0_isr(void) {
  DisableInterrupts disable_interrupts;
  global_spi_instance->HandleInterrupt(disable_interrupts);
}

void porta_isr(void) {
  SpiQueue::global_instance->HandleInterrupt();
}

}  // extern "C"

// A test program which echos characters back after adding a per-UART offset to
// them (CAM0 adds 1, CAM1 adds 2, etc).
__attribute__((unused)) void TestUarts() {
  Uarts *const uarts = Uarts::global_instance;
  while (true) {
    {
      std::array<char, 10> buffer;
      const auto data = uarts->cam0.Read(buffer);
      for (int i = 0; i < data.size(); ++i) {
        data[i] += 1;
      }
      uarts->cam0.Write(data);
    }
    {
      std::array<char, 10> buffer;
      const auto data = uarts->cam1.Read(buffer);
      for (int i = 0; i < data.size(); ++i) {
        data[i] += 2;
      }
      uarts->cam1.Write(data);
    }
    {
      std::array<char, 10> buffer;
      const auto data = uarts->cam2.Read(buffer);
      for (int i = 0; i < data.size(); ++i) {
        data[i] += 3;
      }
      uarts->cam2.Write(data);
    }
    {
      std::array<char, 10> buffer;
      const auto data = uarts->cam3.Read(buffer);
      for (int i = 0; i < data.size(); ++i) {
        data[i] += 4;
      }
      uarts->cam3.Write(data);
    }
    {
      std::array<char, 10> buffer;
      const auto data = uarts->cam4.Read(buffer);
      for (int i = 0; i < data.size(); ++i) {
        data[i] += 5;
      }
      uarts->cam4.Write(data);
    }
  }
}

// Tests all the I/O pins. Cycles through each one for 1 second. While active,
// each output is turned on, and each input has its value printed.
__attribute__((unused)) void TestIo() {
  // Set SPI0 pins to GPIO.
  // SPI_OUT
  PERIPHERAL_BITBAND(GPIOC_PDDR, 6) = 1;
  PORTC_PCR6 = PORT_PCR_DSE | PORT_PCR_MUX(1);
  // SPI_CS
  PERIPHERAL_BITBAND(GPIOD_PDDR, 0) = 0;
  PORTD_PCR0 = PORT_PCR_DSE | PORT_PCR_MUX(1);
  // SPI_IN
  PERIPHERAL_BITBAND(GPIOC_PDDR, 7) = 0;
  PORTC_PCR7 = PORT_PCR_DSE | PORT_PCR_MUX(1);
  // SPI_SCK
  PERIPHERAL_BITBAND(GPIOD_PDDR, 1) = 0;
  PORTD_PCR1 = PORT_PCR_DSE | PORT_PCR_MUX(1);

  // Set LED pins to GPIO.
  PERIPHERAL_BITBAND(GPIOC_PDDR, 11) = 1;
  PORTC_PCR11 = PORT_PCR_DSE | PORT_PCR_MUX(1);
  PERIPHERAL_BITBAND(GPIOC_PDDR, 10) = 1;
  PORTC_PCR10 = PORT_PCR_DSE | PORT_PCR_MUX(1);
  PERIPHERAL_BITBAND(GPIOC_PDDR, 8) = 1;
  PORTC_PCR8 = PORT_PCR_DSE | PORT_PCR_MUX(1);
  PERIPHERAL_BITBAND(GPIOC_PDDR, 9) = 1;
  PORTC_PCR9 = PORT_PCR_DSE | PORT_PCR_MUX(1);
  PERIPHERAL_BITBAND(GPIOB_PDDR, 18) = 1;
  PORTB_PCR18 = PORT_PCR_DSE | PORT_PCR_MUX(1);
  PERIPHERAL_BITBAND(GPIOC_PDDR, 2) = 1;
  PORTC_PCR2 = PORT_PCR_DSE | PORT_PCR_MUX(1);
  PERIPHERAL_BITBAND(GPIOD_PDDR, 7) = 1;
  PORTD_PCR7 = PORT_PCR_DSE | PORT_PCR_MUX(1);
  PERIPHERAL_BITBAND(GPIOC_PDDR, 1) = 1;
  PORTC_PCR1 = PORT_PCR_DSE | PORT_PCR_MUX(1);
  PERIPHERAL_BITBAND(GPIOB_PDDR, 19) = 1;
  PORTB_PCR19 = PORT_PCR_DSE | PORT_PCR_MUX(1);
  PERIPHERAL_BITBAND(GPIOD_PDDR, 5) = 1;
  PORTD_PCR5 = PORT_PCR_DSE | PORT_PCR_MUX(1);

  auto next = aos::monotonic_clock::now();
  static constexpr auto kTick = std::chrono::seconds(1);
  while (true) {
    printf("SPI_MISO\n");
    PERIPHERAL_BITBAND(GPIOC_PDOR, 6) = 1;
    while (aos::monotonic_clock::now() < next + kTick) {
    }
    PERIPHERAL_BITBAND(GPIOC_PDOR, 6) = 0;
    next += kTick;

    while (aos::monotonic_clock::now() < next + kTick) {
      printf("SPI_CS %d\n", (int)PERIPHERAL_BITBAND(GPIOD_PDIR, 0));
    }
    next += kTick;

    while (aos::monotonic_clock::now() < next + kTick) {
      printf("SPI_MOSI %d\n", (int)PERIPHERAL_BITBAND(GPIOC_PDIR, 7));
    }
    next += kTick;

    while (aos::monotonic_clock::now() < next + kTick) {
      printf("SPI_CLK %d\n", (int)PERIPHERAL_BITBAND(GPIOD_PDIR, 1));
    }
    next += kTick;

    printf("CAM0\n");
    PERIPHERAL_BITBAND(GPIOC_PDOR, 11) = 1;
    while (aos::monotonic_clock::now() < next + kTick) {
    }
    PERIPHERAL_BITBAND(GPIOC_PDOR, 11) = 0;
    next += kTick;

    printf("CAM1\n");
    PERIPHERAL_BITBAND(GPIOC_PDOR, 10) = 1;
    while (aos::monotonic_clock::now() < next + kTick) {
    }
    PERIPHERAL_BITBAND(GPIOC_PDOR, 10) = 0;
    next += kTick;

    printf("CAM2\n");
    PERIPHERAL_BITBAND(GPIOC_PDOR, 8) = 1;
    while (aos::monotonic_clock::now() < next + kTick) {
    }
    PERIPHERAL_BITBAND(GPIOC_PDOR, 8) = 0;
    next += kTick;

    printf("CAM3\n");
    PERIPHERAL_BITBAND(GPIOC_PDOR, 9) = 1;
    while (aos::monotonic_clock::now() < next + kTick) {
    }
    PERIPHERAL_BITBAND(GPIOC_PDOR, 9) = 0;
    next += kTick;

    printf("CAM4\n");
    PERIPHERAL_BITBAND(GPIOB_PDOR, 18) = 1;
    while (aos::monotonic_clock::now() < next + kTick) {
    }
    PERIPHERAL_BITBAND(GPIOB_PDOR, 18) = 0;
    next += kTick;

    printf("CAM5\n");
    PERIPHERAL_BITBAND(GPIOC_PDOR, 2) = 1;
    while (aos::monotonic_clock::now() < next + kTick) {
    }
    PERIPHERAL_BITBAND(GPIOC_PDOR, 2) = 0;
    next += kTick;

    printf("CAM6\n");
    PERIPHERAL_BITBAND(GPIOD_PDOR, 7) = 1;
    while (aos::monotonic_clock::now() < next + kTick) {
    }
    PERIPHERAL_BITBAND(GPIOD_PDOR, 7) = 0;
    next += kTick;

    printf("CAM7\n");
    PERIPHERAL_BITBAND(GPIOC_PDOR, 1) = 1;
    while (aos::monotonic_clock::now() < next + kTick) {
    }
    PERIPHERAL_BITBAND(GPIOC_PDOR, 1) = 0;
    next += kTick;

    printf("CAM8\n");
    PERIPHERAL_BITBAND(GPIOB_PDOR, 19) = 1;
    while (aos::monotonic_clock::now() < next + kTick) {
    }
    PERIPHERAL_BITBAND(GPIOB_PDOR, 19) = 0;
    next += kTick;

    printf("CAM9\n");
    PERIPHERAL_BITBAND(GPIOD_PDOR, 5) = 1;
    while (aos::monotonic_clock::now() < next + kTick) {
    }
    PERIPHERAL_BITBAND(GPIOD_PDOR, 5) = 0;
    next += kTick;
  }
}

// Does the normal work of transferring data in all directions.
//
// https://community.nxp.com/thread/466937#comment-983881 is a post from NXP
// claiming that it's impossible to queue up the first byte for the slave end of
// an SPI connection properly. Instead, we just accept there will be a garbage
// byte and the other end ignores it.
__attribute__((unused)) void TransferData() {
  Uarts *const uarts = Uarts::global_instance;
  std::array<CobsPacketizer<uart_to_teensy_size()>, 5> packetizers;
  std::array<TransmitBuffer, 5> transmit_buffers{
      {&uarts->cam0, &uarts->cam1, &uarts->cam2, &uarts->cam3, &uarts->cam4}};
  FrameQueue frame_queue;
  aos::monotonic_clock::time_point last_camera_send =
      aos::monotonic_clock::min_time;
  bool first = true;
  while (true) {
    {
      const auto received_transfer = SpiQueue::global_instance->Tick();
      if (received_transfer) {
        const auto unpacked = SpiUnpackToTeensy(*received_transfer);
        if (!unpacked) {
          printf("UART decode error\n");
        }
      }
    }

    {
      std::array<char, 20> buffer;
      packetizers[0].ParseData(uarts->cam0.Read(buffer));
      packetizers[1].ParseData(uarts->cam1.Read(buffer));
      packetizers[2].ParseData(uarts->cam2.Read(buffer));
      packetizers[3].ParseData(uarts->cam3.Read(buffer));
      packetizers[4].ParseData(uarts->cam4.Read(buffer));
    }
    for (size_t i = 0; i < packetizers.size(); ++i) {
      if (!packetizers[i].received_packet().empty()) {
        const auto decoded =
            UartUnpackToTeensy(packetizers[i].received_packet());
        packetizers[i].clear_received_packet();
        if (decoded) {
          printf("got one with %d\n", (int)decoded->targets.size());
          frame_queue.UpdateFrame(i, *decoded);
        }
      }
    }
    {
      bool made_transfer = false;
      if (!first) {
        DisableInterrupts disable_interrupts;
        made_transfer =
            !SpiQueue::global_instance->HaveTransfer(disable_interrupts);
      }
      if (made_transfer) {
        frame_queue.RemoveLatestFrames();
      }
      const auto transfer = frame_queue.MakeTransfer();
      {
        DisableInterrupts disable_interrupts;
        SpiQueue::global_instance->UpdateTransfer(transfer, disable_interrupts);
      }
    }
    {
      const auto now = aos::monotonic_clock::now();
      if (last_camera_send + std::chrono::milliseconds(1000) < now) {
        last_camera_send = now;
        CameraCalibration calibration{};
        calibration.teensy_now = aos::monotonic_clock::now();
        calibration.realtime_now = aos::realtime_clock::min_time;
        calibration.camera_command = CameraCommand::kNormal;
        // TODO(Brian): Actually fill out the calibration field.
        transmit_buffers[0].MaybeWritePacket(calibration);
        transmit_buffers[1].MaybeWritePacket(calibration);
        transmit_buffers[2].MaybeWritePacket(calibration);
        transmit_buffers[3].MaybeWritePacket(calibration);
        transmit_buffers[4].MaybeWritePacket(calibration);
      }
      for (TransmitBuffer &transmit_buffer : transmit_buffers) {
        transmit_buffer.Tick(now);
      }
    }

    first = false;
  }
}

int Main() {
  // for background about this startup delay, please see these conversations
  // https://forum.pjrc.com/threads/36606-startup-time-(400ms)?p=113980&viewfull=1#post113980
  // https://forum.pjrc.com/threads/31290-Teensey-3-2-Teensey-Loader-1-24-Issues?p=87273&viewfull=1#post87273
  delay(400);

  // Set all interrupts to the second-lowest priority to start with.
  for (int i = 0; i < NVIC_NUM_INTERRUPTS; i++) NVIC_SET_SANE_PRIORITY(i, 0xD);

  // Now set priorities for all the ones we care about. They only have meaning
  // relative to each other, which means centralizing them here makes it a lot
  // more manageable.
  NVIC_SET_SANE_PRIORITY(IRQ_USBOTG, 0x7);
  NVIC_SET_SANE_PRIORITY(IRQ_UART0_STATUS, 0x3);
  NVIC_SET_SANE_PRIORITY(IRQ_UART1_STATUS, 0x3);
  NVIC_SET_SANE_PRIORITY(IRQ_UART2_STATUS, 0x3);
  NVIC_SET_SANE_PRIORITY(IRQ_UART3_STATUS, 0x3);
  NVIC_SET_SANE_PRIORITY(IRQ_UART4_STATUS, 0x3);
  // This one is relatively sensitive to latencies. The buffer is ~4800 clock
  // cycles long.
  NVIC_SET_SANE_PRIORITY(IRQ_SPI0, 0x2);
  NVIC_SET_SANE_PRIORITY(IRQ_PORTA, 0x3);

  // Set the LED's pin to output mode.
  PERIPHERAL_BITBAND(GPIOC_PDDR, 5) = 1;
  PORTC_PCR5 = PORT_PCR_DSE | PORT_PCR_MUX(1);

  frc971::motors::PrintingParameters printing_parameters;
  printing_parameters.dedicated_usb = true;
  const ::std::unique_ptr<frc971::motors::PrintingImplementation> printing =
      CreatePrinting(printing_parameters);
  printing->Initialize();

  DMA.CR = M_DMA_EMLM;

  SIM_SCGC1 |= SIM_SCGC1_UART4;
  SIM_SCGC4 |=
      SIM_SCGC4_UART0 | SIM_SCGC4_UART1 | SIM_SCGC4_UART2 | SIM_SCGC4_UART3;
  SIM_SCGC6 |= SIM_SCGC6_SPI0;

  // SPI0 goes to the roboRIO.
  // SPI0_PCS0 is SPI_CS.
  PORTD_PCR0 = PORT_PCR_MUX(2);
  // SPI0_SOUT is SPI_MISO.
  PORTC_PCR6 = PORT_PCR_DSE | PORT_PCR_MUX(2);
  // SPI0_SIN is SPI_MOSI.
  PORTC_PCR7 = PORT_PCR_DSE | PORT_PCR_MUX(2);
  // SPI0_SCK is SPI_CLK.
  PORTD_PCR1 = PORT_PCR_DSE | PORT_PCR_MUX(2);
  // SPI_CS_DRIVE
  PERIPHERAL_BITBAND(GPIOB_PDDR, 17) = 1;
  PERIPHERAL_BITBAND(GPIOB_PDOR, 17) = 1;
  PORTB_PCR17 = PORT_PCR_DSE | PORT_PCR_MUX(1);
  // SPI_CS_IN
  PERIPHERAL_BITBAND(GPIOA_PDDR, 17) = 0;
  // Set the filter width.
  PORTA_DFWR = 31;
  // Enable the filter.
  PERIPHERAL_BITBAND(PORTA_DFER, 17) = 1;
  PORTA_PCR17 =
      PORT_PCR_MUX(1) | PORT_PCR_IRQC(0xC) /* Interrupt when logic 1 */;
  // Clear the interrupt flag now that we've reconfigured it.
  PORTA_ISFR = 1 << 17;

  // FTM0_CH0 is LED0 (7 in silkscreen, a beacon channel).
  PORTC_PCR1 = PORT_PCR_DSE | PORT_PCR_MUX(4);
  // FTM0_CH1 is LED1 (5 in silkscreen, a beacon channel).
  PORTC_PCR2 = PORT_PCR_DSE | PORT_PCR_MUX(4);
  // FTM0_CH7 is LED2 (6 in silkscreen, a beacon channel).
  PORTD_PCR7 = PORT_PCR_DSE | PORT_PCR_MUX(4);
  // FTM0_CH5 is LED3 (9 in silkscreen, a vision camera).
  PORTD_PCR5 = PORT_PCR_DSE | PORT_PCR_MUX(4);

  // FTM2_CH1 is LED4 (8 in silkscreen, a vision camera).
  PORTB_PCR19 = PORT_PCR_DSE | PORT_PCR_MUX(3);
  // FTM2_CH0 is LED5 (for CAM4).
  PORTB_PCR18 = PORT_PCR_DSE | PORT_PCR_MUX(3);

  // FTM3_CH4 is LED6 (for CAM2).
  PORTC_PCR8 = PORT_PCR_DSE | PORT_PCR_MUX(3);
  // FTM3_CH5 is LED7 (for CAM3).
  PORTC_PCR9 = PORT_PCR_DSE | PORT_PCR_MUX(3);
  // FTM3_CH6 is LED8 (for CAM1).
  PORTC_PCR10 = PORT_PCR_DSE | PORT_PCR_MUX(3);
  // FTM3_CH7 is LED9 (for CAM0).
  PORTC_PCR11 = PORT_PCR_DSE | PORT_PCR_MUX(3);

  // This hardware has been deactivated, but keep this comment for now to
  // document which pins it is on.
#if 0
  // This is ODROID_EN.
  PERIPHERAL_BITBAND(GPIOC_PDDR, 0) = 1;
  PERIPHERAL_BITBAND(GPIOC_PDOR, 0) = 0;
  PORTC_PCR0 = PORT_PCR_DSE | PORT_PCR_MUX(1);
  // This is CAM_EN.
  PERIPHERAL_BITBAND(GPIOB_PDDR, 0) = 1;
  PERIPHERAL_BITBAND(GPIOB_PDOR, 0) = 0;
  PORTB_PCR0 = PORT_PCR_DSE | PORT_PCR_MUX(1);
#endif
  // This is 5V_PGOOD.
  PERIPHERAL_BITBAND(GPIOD_PDDR, 6) = 0;
  PORTD_PCR6 = PORT_PCR_MUX(1);

  // These go to CAM1.
  // UART0_RX (peripheral) is UART1_RX (schematic).
  PORTA_PCR15 = PORT_PCR_DSE | PORT_PCR_MUX(3) | PORT_PCR_PE /* Do a pull */ |
                0 /* !PS to pull down */;
  // UART0_TX (peripheral) is UART1_TX (schematic).
  PORTA_PCR14 = PORT_PCR_DSE | PORT_PCR_MUX(3);

  // These go to CAM0.
  // UART1_RX (peripheral) is UART0_RX (schematic).
  PORTC_PCR3 = PORT_PCR_DSE | PORT_PCR_MUX(3) | PORT_PCR_PE /* Do a pull */ |
               0 /* !PS to pull down */;
  // UART1_TX (peripheral) is UART0_TX (schematic).
  PORTC_PCR4 = PORT_PCR_DSE | PORT_PCR_MUX(3);

  // These go to CAM2.
  // UART2_RX
  PORTD_PCR2 = PORT_PCR_DSE | PORT_PCR_MUX(3) | PORT_PCR_PE /* Do a pull */ |
               0 /* !PS to pull down */;
  // UART2_TX
  PORTD_PCR3 = PORT_PCR_DSE | PORT_PCR_MUX(3);

  // These go to CAM3.
  // UART3_RX
  PORTB_PCR10 = PORT_PCR_DSE | PORT_PCR_MUX(3) | PORT_PCR_PE /* Do a pull */ |
                0 /* !PS to pull down */;
  // UART3_TX
  PORTB_PCR11 = PORT_PCR_DSE | PORT_PCR_MUX(3);

  // These go to CAM4.
  // UART4_RX
  PORTE_PCR25 = PORT_PCR_DSE | PORT_PCR_MUX(3) | PORT_PCR_PE /* Do a pull */ |
                0 /* !PS to pull down */;
  // UART4_TX
  PORTE_PCR24 = PORT_PCR_DSE | PORT_PCR_MUX(3);

  Uarts uarts;
  InterruptBufferedSpi spi0{&SPI0, BUS_CLOCK_FREQUENCY};
  global_spi_instance = &spi0;
  SpiQueue spi_queue;

  // Give everything a chance to get going.
  delay(100);

  printf("Ram start:   %p\n", __bss_ram_start__);
  printf("Heap start:  %p\n", __heap_start__);
  printf("Heap end:    %p\n", __brkval);
  printf("Stack start: %p\n", __stack_end__);

  uarts.Initialize(115200);
  NVIC_ENABLE_IRQ(IRQ_UART0_STATUS);
  NVIC_ENABLE_IRQ(IRQ_UART1_STATUS);
  NVIC_ENABLE_IRQ(IRQ_UART2_STATUS);
  NVIC_ENABLE_IRQ(IRQ_UART3_STATUS);
  NVIC_ENABLE_IRQ(IRQ_UART4_STATUS);
  spi0.Initialize();
  NVIC_ENABLE_IRQ(IRQ_SPI0);
  NVIC_ENABLE_IRQ(IRQ_PORTA);

  TransferData();

  while (true) {
  }
}

extern "C" {

int main(void) {
  return Main();
}

}  // extern "C"

}  // namespace
}  // namespace jevois
}  // namespace frc971
