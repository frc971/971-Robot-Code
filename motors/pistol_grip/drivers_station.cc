// This file has the main for the Teensy in the driver's station that
// communicates over CAN with the one in the pistol grip controller.

#include <stdio.h>
#include <atomic>

#include "motors/core/kinetis.h"
#include "motors/core/time.h"
#include "motors/peripheral/can.h"
#include "motors/print/print.h"
#include "motors/usb/cdc.h"
#include "motors/usb/hid.h"
#include "motors/usb/interrupt_out.h"
#include "motors/usb/usb.h"
#include "motors/util.h"

namespace frc971 {
namespace motors {
namespace {

// TODO(Brian): Move this and the other two test functions somewhere else.
__attribute__((unused)) void EchoChunks(teensy::AcmTty *tty1) {
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

__attribute__((unused)) void EchoImmediately(teensy::AcmTty *tty1) {
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

__attribute__((unused)) void WriteData(teensy::AcmTty *tty1) {
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

void ForwardJoystickData(teensy::HidFunction *throttle_joystick,
                         teensy::HidFunction *wheel_joystick,
                         teensy::InterruptOut *interrupt_out) {
  uint32_t last_command_time = micros();
  uint16_t trigger_position = 0x8000;
  uint16_t wheel_position = 0x8000;
  uint16_t trigger_velocity = 0x8000;
  uint16_t wheel_velocity = 0x8000;
  uint16_t trigger_torque = 0x8000;
  uint16_t wheel_torque = 0x8000;
  uint16_t buttons = 0x0080;
  while (true) {
    bool update_report = false;

    uint8_t can_data[8];
    int length;
    can_receive(can_data, &length, 0);
    if (length == 8) {
      last_command_time = micros();
      trigger_position =
          static_cast<uint16_t>(static_cast<uint32_t>(can_data[0]) |
                                (static_cast<uint32_t>(can_data[1]) << 8));
      trigger_velocity =
          static_cast<uint16_t>(static_cast<uint32_t>(can_data[2]) |
                                (static_cast<uint32_t>(can_data[3]) << 8));
      trigger_torque =
          static_cast<uint16_t>(static_cast<uint32_t>(can_data[4]) |
                                (static_cast<uint32_t>(can_data[5]) << 8));

      buttons = static_cast<uint16_t>(
          (buttons & 0xc) | (static_cast<uint32_t>(can_data[7] & 0xc0) >> 6));
      update_report = true;
    }

    can_receive(can_data, &length, 1);
    if (length == 8) {
      last_command_time = micros();
      wheel_position =
          static_cast<uint16_t>(static_cast<uint32_t>(can_data[0]) |
                                (static_cast<uint32_t>(can_data[1]) << 8));
      wheel_velocity =
          static_cast<uint16_t>(static_cast<uint32_t>(can_data[2]) |
                                (static_cast<uint32_t>(can_data[3]) << 8));
      wheel_torque =
          static_cast<uint16_t>(static_cast<uint32_t>(can_data[4]) |
                                (static_cast<uint32_t>(can_data[5]) << 8));

      buttons = static_cast<uint16_t>(
          (buttons & 0x3) | (static_cast<uint32_t>(can_data[7] & 0xc0) >> 4));
      update_report = true;
    }

    static constexpr uint32_t kTimeout = 10000;
    if (!time_after(time_add(last_command_time, kTimeout), micros())) {
      trigger_position = 0x8000;
      wheel_position = 0x8000;
      trigger_velocity = 0x8000;
      wheel_velocity = 0x8000;
      trigger_torque = 0x8000;
      wheel_torque = 0x8000;
      buttons = 0x0080;
      update_report = true;
      // Avoid wrapping back into the valid range.
      last_command_time = time_subtract(micros(), kTimeout);
    }

    if (update_report) {
      DisableInterrupts disable_interrupts;

      const uint16_t trigger_packet[] = {
          trigger_position,
          trigger_velocity,
          trigger_torque,
          static_cast<uint16_t>((trigger_position & 0xff) << 8),
          static_cast<uint16_t>((trigger_velocity & 0xff) << 8),
          static_cast<uint16_t>((trigger_torque & 0xff) << 8),
          buttons};
      throttle_joystick->UpdateReport(trigger_packet, 14, disable_interrupts);

      const uint16_t wheel_packet[] = {
          wheel_position,
          wheel_velocity,
          wheel_torque,
          static_cast<uint16_t>((wheel_position & 0xff) << 8),
          static_cast<uint16_t>((wheel_velocity & 0xff) << 8),
          static_cast<uint16_t>((wheel_torque & 0xff) << 8),
          buttons};
      wheel_joystick->UpdateReport(wheel_packet, 14, disable_interrupts);
    }

    char usb_out_data[teensy::InterruptOut::kSize];
    const int usb_out_size = interrupt_out->ReceiveData(usb_out_data);
    if (usb_out_size >= 16) {
      can_send(0x2, reinterpret_cast<unsigned char *>(usb_out_data), 8, 2);
      can_send(0x3, reinterpret_cast<unsigned char *>(usb_out_data) + 8, 8, 3);
    }
  }
}

// The HID report descriptor we use.
constexpr char kReportDescriptor[] = {
    0x05, 0x01,        // Usage Page (Generic Desktop),
    0x09, 0x04,        // Usage (Joystick),
    0xA1, 0x01,        // Collection (Application),
    0x75, 0x10,        //     Report Size (16),
    0x95, 0x06,        //     Report Count (6),
    0x15, 0x00,        //     Logical Minimum (0),
    0x26, 0xFF, 0xFF,  //     Logical Maximum (65535),
    0x35, 0x00,        //     Physical Minimum (0),
    0x46, 0xFF, 0xFF,  //     Physical Maximum (65535),
    0x09, 0x30,        //     Usage (X),
    0x09, 0x31,        //     Usage (Y),
    0x09, 0x32,        //     Usage (Z),
    0x09, 0x33,        //     Usage (Rz),
    0x09, 0x34,        //     Usage (?),
    0x09, 0x35,        //     Usage (?),
    0x81, 0x02,        //     Input (Variable),
    0x75, 0x01,        //     Report Size (1),
    0x95, 0x10,        //     Report Count (16),
    0x25, 0x01,        //     Logical Maximum (1),
    0x45, 0x01,        //     Physical Maximum (1),
    0x05, 0x09,        //     Usage Page (Button),
    0x19, 0x01,        //     Usage Minimum (01),
    0x29, 0x10,        //     Usage Maximum (16),
    0x81, 0x02,        //     Input (Variable),
    0xC0               // End Collection
};

}  // namespace

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
  PERIPHERAL_BITBAND(GPIOC_PDDR, 5) = 1;
  PORTC_PCR5 = PORT_PCR_DSE | PORT_PCR_MUX(1);

  // Set up the CAN pins.
#if 0
  // Pistol grip motor controller board.
  PORTA_PCR12 = PORT_PCR_DSE | PORT_PCR_MUX(2);
  PORTA_PCR13 = PORT_PCR_DSE | PORT_PCR_MUX(2);
#else
  // Button board.
  // TODO(austin): Drive this based off the processor ID.
  PORTB_PCR18 = PORT_PCR_DSE | PORT_PCR_MUX(2);
  PORTB_PCR19 = PORT_PCR_DSE | PORT_PCR_MUX(2);
#endif

  delay(100);

  teensy::UsbDevice usb_device(0, 0x16c0, 0x0491);
  usb_device.SetManufacturer("FRC 971 Spartan Robotics");
  usb_device.SetProduct("Pistol Grip Controller interface");

  teensy::HidFunction throttle_joystick(&usb_device, 14);
  throttle_joystick.set_report_descriptor(
      ::std::string(kReportDescriptor, sizeof(kReportDescriptor)));

  teensy::HidFunction wheel_joystick(&usb_device, 14);
  wheel_joystick.set_report_descriptor(
      ::std::string(kReportDescriptor, sizeof(kReportDescriptor)));

  teensy::AcmTty tty1(&usb_device);
  teensy::AcmTty tty2(&usb_device);
  teensy::InterruptOut interrupt_out(&usb_device, "JoystickForce");
  PrintingParameters printing_parameters;
  printing_parameters.stdout_tty = &tty1;
  printing_parameters.debug_tty = &tty2;
  const ::std::unique_ptr<PrintingImplementation> printing =
      CreatePrinting(printing_parameters);

  usb_device.Initialize();
  printing->Initialize();

  can_init(0, 1);

  ForwardJoystickData(&throttle_joystick, &wheel_joystick, &interrupt_out);

  return 0;
}

}  // namespace motors
}  // namespace frc971
