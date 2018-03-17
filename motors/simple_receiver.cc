// This file has the main for the Teensy on the simple receiver board.

#include <inttypes.h>
#include <stdio.h>
#include <atomic>
#include <cmath>

#include "motors/core/time.h"
#include "motors/core/kinetis.h"
#include "motors/peripheral/adc.h"
#include "motors/peripheral/can.h"
#include "motors/usb/usb.h"
#include "motors/usb/cdc.h"
#include "motors/util.h"

namespace frc971 {
namespace motors {
namespace {

::std::atomic<teensy::AcmTty *> global_stdout{nullptr};

}  // namespace

extern "C" {

void *__stack_chk_guard = (void *)0x67111971;

int _write(int /*file*/, char *ptr, int len) {
  teensy::AcmTty *const tty = global_stdout.load(::std::memory_order_acquire);
  if (tty != nullptr) {
    return tty->Write(ptr, len);
  }
  return 0;
}

void __stack_chk_fail(void);

void DoTest() {
  uint32_t time = micros();
  while (true) {
    for (int i = 0; i < 6; ++i) {
      const uint32_t end = time_add(time, 500000);
      while (true) {
        const bool done = time_after(micros(), end);
        double current;
        if (done) {
          current = -6;
        } else {
          current = 6;
        }
        const int32_t current_int = current * 1000;
        uint32_t id = CAN_EFF_FLAG;
        id |= i;
        id |= (0x01 /* SET_CURRENT */) << 8;
        uint8_t data[4] = {
            static_cast<uint8_t>((current_int >> 24) & 0xFF),
            static_cast<uint8_t>((current_int >> 16) & 0xFF),
            static_cast<uint8_t>((current_int >> 8) & 0xFF),
            static_cast<uint8_t>((current_int >> 0) & 0xFF),
        };
        can_send(id, data, sizeof(data), 2);
        if (done) {
          break;
        }
        delay(5);
      }
      time = end;
    }
  }
}

}  // extern "C"

extern "C" int main(void) {
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

  // Builtin LED.
  PERIPHERAL_BITBAND(GPIOC_PDOR, 5) = 1;
  PORTC_PCR5 = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(1);
  PERIPHERAL_BITBAND(GPIOC_PDDR, 5) = 1;

  // Set up the CAN pins.
  PORTA_PCR12 = PORT_PCR_DSE | PORT_PCR_MUX(2);
  PORTA_PCR13 = PORT_PCR_DSE | PORT_PCR_MUX(2);

  delay(100);

  teensy::UsbDevice usb_device(0, 0x16c0, 0x0492);
  usb_device.SetManufacturer("Seems Reasonable LLC");
  usb_device.SetProduct("Simple Receiver Board");

  teensy::AcmTty tty0(&usb_device);
  global_stdout.store(&tty0, ::std::memory_order_release);
  usb_device.Initialize();

  can_init(0, 1);
  salsa::AdcInitJoystick();

  // Leave the LEDs on for a bit longer.
  delay(300);
  printf("Done starting up\n");

  // Done starting up, now turn the LED off.
  PERIPHERAL_BITBAND(GPIOC_PDOR, 5) = 0;

  DoTest();

  return 0;
}

void __stack_chk_fail(void) {
  while (true) {
    GPIOC_PSOR = (1 << 5);
    printf("Stack corruption detected\n");
    delay(1000);
    GPIOC_PCOR = (1 << 5);
    delay(1000);
  }
}

}  // namespace motors
}  // namespace frc971
