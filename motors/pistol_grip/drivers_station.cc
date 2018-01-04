// This file has the main for the Teensy in the driver's station that
// communicates over CAN with the one in the pistol grip controller.

#include <stdio.h>
#include <atomic>

#include "motors/core/time.h"
#include "motors/core/kinetis.h"
#include "motors/usb/usb.h"
#include "motors/usb/cdc.h"
#include "motors/usb/hid.h"
#include "motors/util.h"

namespace frc971 {
namespace motors {
namespace {

::std::atomic<teensy::AcmTty *> global_stdout{nullptr};

void EchoChunks(teensy::AcmTty *tty1) {
  while (true) {
    char buffer[512];
    size_t buffered = 0;
    while (buffered < sizeof(buffer)) {
      const size_t chunk =
          tty1->Read(&buffer[buffered], sizeof(buffer) - buffered);
      buffered += chunk;
    }
    size_t written = 0;
    while (written < buffered) {
      const size_t chunk = tty1->Write(&buffer[written], buffered - written);
      written += chunk;
    }

    GPIOC_PTOR = 1 << 5;
    for (int i = 0; i < 100000000; ++i) {
      GPIOC_PSOR = 0;
    }
    GPIOC_PTOR = 1 << 5;
  }
}

void EchoImmediately(teensy::AcmTty *tty1) {
  while (true) {
    if (false) {
      // Delay for a while.
      for (int i = 0; i < 100000000; ++i) {
        GPIOC_PSOR = 0;
      }
    }

    char buffer[64];
    const size_t chunk = tty1->Read(buffer, sizeof(buffer));
    size_t written = 0;
    while (written < chunk) {
      written += tty1->Write(&buffer[written], chunk - written);
    }
  }
}

void WriteData(teensy::AcmTty *tty1) {
  GPIOC_PTOR = 1 << 5;
  for (int i = 0; i < 100000000; ++i) {
    GPIOC_PSOR = 0;
  }
  GPIOC_PTOR = 1 << 5;
  for (int i = 0; i < 100000000; ++i) {
    GPIOC_PSOR = 0;
  }

  const char data[] =
      "Running command lineRunning command lineRunning command lineRunning "
      "command lineRunning command lineRunning command lineRunning command "
      "lineRunning command lineRunning command lineRunning command lineRunning "
      "command lineRunning command lineRunning command lineRunning command "
      "lineRunning command lineRunning command lineRunning command lineRunning "
      "command lineRunning command lineRunning command lineRunning command "
      "lineRunning command lineRunning command lineRunning command lineRunning "
      "command lineRunning command lineRunning command lineRunning command "
      "lineRunning command lineRunning command lineRunning command lineRunning "
      "command lineRunning command lineRunning command lineRunning command "
      "lineRunning command lineRunning command lineRunning command lineRunning "
      "command lineRunning command lineRunning command lineRunning command "
      "lineRunning command lineRunning command lineRunning command lineRunning "
      "command lineRunning command lineRunning command lineRunning command "
      "lineRunning command lineRunning command lineRunning command lineRunning "
      "command lineRunning command lineRunning command lineRunning command "
      "lineRunning command lineRunning command lineRunning command lineRunning "
      "command lineRunning command lineRunning command lineRunning command "
      "lineRunning command line\n";
  size_t written = 0;
  while (written < sizeof(data)) {
    written += tty1->Write(&data[written], sizeof(data) - written);
  }
  GPIOC_PSOR = 1 << 5;
  while (true) {
  }
}

void MoveJoysticks(teensy::HidFunction *joysticks) {
  static uint8_t x_axis = 0, y_axis = 97, z_axis = 5, rz_axis = 8;
  while (true) {
    {
      DisableInterrupts disable_interrupts;
      uint8_t buttons = 0;
      if (x_axis % 8u) {
        buttons = 2;
      }
      uint8_t report[10] = {x_axis,  0, y_axis, 0,       z_axis,
                            rz_axis, 0, 0,      buttons, 0};
      joysticks->UpdateReport(report, 10, disable_interrupts);
    }
    delay(1);
    x_axis += 1;
    y_axis += 3;
    z_axis += 5;
    rz_axis += 8;
  }
}

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

  // Set the LED's pin to output mode.
  GPIO_BITBAND(GPIOC_PDDR, 5) = 1;
  PORTC_PCR5 = PORT_PCR_DSE | PORT_PCR_MUX(1);

  delay(100);

  teensy::UsbDevice usb_device(0, 0x16c0, 0x0492);
  usb_device.SetManufacturer("FRC 971 Spartan Robotics");
  usb_device.SetProduct("Pistol Grip Controller interface");
  teensy::HidFunction joysticks(&usb_device, 10);
  // TODO(Brian): Figure out why Windows refuses to recognize the joystick along
  // with a TTY or two.
#if 0
  teensy::AcmTty tty1(&usb_device);
  teensy::AcmTty tty2(&usb_device);
  global_stdout.store(&tty2, ::std::memory_order_release);
#endif
  usb_device.Initialize();

  MoveJoysticks(&joysticks);

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
