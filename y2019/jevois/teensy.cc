#include "aos/time/time.h"
#include "motors/core/kinetis.h"
#include "motors/core/time.h"
#include "motors/peripheral/configuration.h"
#include "motors/peripheral/uart.h"
#include "motors/print/print.h"
#include "motors/util.h"

namespace frc971 {
namespace jevois {
namespace {

struct Uarts {
  Uarts() {
    DisableInterrupts disable_interrupts;
    instance = this;
  }
  ~Uarts() {
    DisableInterrupts disable_interrupts;
    instance = nullptr;
  }

  void Initialize(int baud_rate) {
    cam0.Initialize(baud_rate);
    cam1.Initialize(baud_rate);
    cam2.Initialize(baud_rate);
    cam3.Initialize(baud_rate);
    cam4.Initialize(baud_rate);
  }

  frc971::teensy::InterruptBufferedUart cam0{&UART1, F_CPU};
  frc971::teensy::InterruptBufferedUart cam1{&UART0, F_CPU};
  frc971::teensy::InterruptBufferedUart cam2{&UART2, BUS_CLOCK_FREQUENCY};
  frc971::teensy::InterruptBufferedUart cam3{&UART3, BUS_CLOCK_FREQUENCY};
  frc971::teensy::InterruptBufferedUart cam4{&UART4, BUS_CLOCK_FREQUENCY};

  static Uarts *instance;
};

Uarts *Uarts::instance = nullptr;

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
  Uarts::instance->cam1.HandleInterrupt(disable_interrupts);
}

void uart1_status_isr(void) {
  DisableInterrupts disable_interrupts;
  Uarts::instance->cam0.HandleInterrupt(disable_interrupts);
}

void uart2_status_isr(void) {
  DisableInterrupts disable_interrupts;
  Uarts::instance->cam2.HandleInterrupt(disable_interrupts);
}

void uart3_status_isr(void) {
  DisableInterrupts disable_interrupts;
  Uarts::instance->cam3.HandleInterrupt(disable_interrupts);
}

void uart4_status_isr(void) {
  DisableInterrupts disable_interrupts;
  Uarts::instance->cam4.HandleInterrupt(disable_interrupts);
}

}  // extern "C"

// A test program which echos characters back after adding a per-UART offset to
// them (CAM0 adds 1, CAM1 adds 2, etc).
__attribute__((unused)) void TestUarts() {
  Uarts *const uarts = Uarts::instance;
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
  NVIC_SET_SANE_PRIORITY(IRQ_FTM0, 0x3);
  NVIC_SET_SANE_PRIORITY(IRQ_UART0_STATUS, 0xE);

  // Set the LED's pin to output mode.
  PERIPHERAL_BITBAND(GPIOC_PDDR, 5) = 1;
  PORTC_PCR5 = PORT_PCR_DSE | PORT_PCR_MUX(1);

  frc971::motors::PrintingParameters printing_parameters;
  printing_parameters.dedicated_usb = true;
  const ::std::unique_ptr<frc971::motors::PrintingImplementation> printing =
      CreatePrinting(printing_parameters);
  printing->Initialize();

  DMA.CR = M_DMA_EMLM;

  SIM_SCGC4 |=
      SIM_SCGC4_UART0 | SIM_SCGC4_UART1 | SIM_SCGC4_UART2 | SIM_SCGC4_UART3;
  SIM_SCGC1 |= SIM_SCGC1_UART4;

  // SPI0 goes to the roboRIO.
  // SPI0_PCS0 is SPI_CS.
  PORTD_PCR0 = PORT_PCR_DSE | PORT_PCR_MUX(2);
  // SPI0_SOUT is SPI_MISO.
  PORTC_PCR6 = PORT_PCR_DSE | PORT_PCR_MUX(2);
  // SPI0_SIN is SPI_MOSI.
  PORTC_PCR7 = PORT_PCR_DSE | PORT_PCR_MUX(2);
  // SPI0_SCK is SPI_CLK.
  PORTD_PCR1 = PORT_PCR_DSE | PORT_PCR_MUX(2);

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
  PORTA_PCR15 = PORT_PCR_DSE | PORT_PCR_MUX(3);
  // UART0_TX (peripheral) is UART1_TX (schematic).
  PORTA_PCR14 = PORT_PCR_DSE | PORT_PCR_MUX(3);

  // These go to CAM0.
  // UART1_RX (peripheral) is UART0_RX (schematic).
  PORTC_PCR3 = PORT_PCR_DSE | PORT_PCR_MUX(3);
  // UART1_TX (peripheral) is UART0_TX (schematic).
  PORTC_PCR4 = PORT_PCR_DSE | PORT_PCR_MUX(3);

  // These go to CAM2.
  // UART2_RX
  PORTD_PCR2 = PORT_PCR_DSE | PORT_PCR_MUX(3);
  // UART2_TX
  PORTD_PCR3 = PORT_PCR_DSE | PORT_PCR_MUX(3);

  // These go to CAM3.
  // UART3_RX
  PORTB_PCR10 = PORT_PCR_DSE | PORT_PCR_MUX(3);
  // UART3_TX
  PORTB_PCR11 = PORT_PCR_DSE | PORT_PCR_MUX(3);

  // These go to CAM4.
  // UART4_RX
  PORTE_PCR25 = PORT_PCR_DSE | PORT_PCR_MUX(3);
  // UART4_TX
  PORTE_PCR24 = PORT_PCR_DSE | PORT_PCR_MUX(3);

  Uarts uarts;

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
