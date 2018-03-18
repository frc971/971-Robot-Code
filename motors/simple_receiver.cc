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

void DoVescTest() {
  uint32_t time = micros();
  while (true) {
    for (int i = 0; i < 6; ++i) {
      const uint32_t end = time_add(time, 500000);
      while (true) {
        const bool done = time_after(micros(), end);
        float current;
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

void DoReceiverTest() {
  while (true) {
    FTM0->STATUS = 0x00;
    while (!(FTM0->STATUS & (1 << 1))) {}
    const uint32_t start = FTM0->C0V;
    const uint32_t end = FTM0->C1V;
    const uint32_t now = micros();
    const uint32_t width = (end - start) & 0xFFFF;
    printf("got pulse %" PRIu32 "-%" PRIu32 "=%" PRIu32 " at %" PRIu32 "\n",
           start, end, width, now);

    for (int i = 0; i < 6; ++i) {
      // 4290 - 6966
      // 4133 - 7117
#if 0
      float current =
          static_cast<float>(static_cast<int>(width) - 5625) / 1338.0f * 4.0f;
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
#else
      float rpm = static_cast<float>(static_cast<int>(width) - 5625) / 1338.0f *
                  10000.0f;
      const int32_t rpm_int = rpm;
      uint32_t id = CAN_EFF_FLAG;
      id |= i;
      id |= (0x03 /* SET_RPM */) << 8;
      uint8_t data[4] = {
          static_cast<uint8_t>((rpm_int >> 24) & 0xFF),
          static_cast<uint8_t>((rpm_int >> 16) & 0xFF),
          static_cast<uint8_t>((rpm_int >> 8) & 0xFF),
          static_cast<uint8_t>((rpm_int >> 0) & 0xFF),
      };
#endif
      can_send(id, data, sizeof(data), 2);
      delay(1);
    }
  }
}

// 4290 - 6966
// 4133 - 7117
void DoReceiverTest2() {
  static constexpr int kMin = 4133, kMax = 7177;
  static constexpr int kMid = (kMin + kMax) / 2, kRange = kMax - kMin;
  static constexpr float kMaxRpm = 10000.0f;
  while (true) {
    FTM0->STATUS = 0x00;

    while (!(FTM0->STATUS & (1 << 5))) {
    }
    const uint32_t flip_start = FTM0->C4V;
    const uint32_t flip_end = FTM0->C5V;
    const bool flip = ((flip_end - flip_start) & 0xFFFF) > kMid;

    while (!(FTM0->STATUS & (1 << 1))) {
    }
    {
      const uint32_t start = FTM0->C0V;
      const uint32_t end = FTM0->C1V;
      const int width = (end - start) & 0xFFFF;

      {
        float rpm = static_cast<float>(::std::min(0, width - kMid)) /
                    static_cast<float>(kRange) * kMaxRpm;
        if (flip) {
          rpm *= -1.0f;
        }
        const int32_t rpm_int = rpm;
        uint32_t id = CAN_EFF_FLAG;
        id |= 0;
        id |= (0x03 /* SET_RPM */) << 8;
        uint8_t data[4] = {
            static_cast<uint8_t>((rpm_int >> 24) & 0xFF),
            static_cast<uint8_t>((rpm_int >> 16) & 0xFF),
            static_cast<uint8_t>((rpm_int >> 8) & 0xFF),
            static_cast<uint8_t>((rpm_int >> 0) & 0xFF),
        };
        can_send(id, data, sizeof(data), 2);
        delay(1);
      }

      {
        float rpm = static_cast<float>(::std::max(0, width - kMid)) /
                    static_cast<float>(kRange) * kMaxRpm;
        if (flip) {
          rpm *= -1.0f;
        }
        const int32_t rpm_int = rpm;
        uint32_t id = CAN_EFF_FLAG;
        id |= 1;
        id |= (0x03 /* SET_RPM */) << 8;
        uint8_t data[4] = {
            static_cast<uint8_t>((rpm_int >> 24) & 0xFF),
            static_cast<uint8_t>((rpm_int >> 16) & 0xFF),
            static_cast<uint8_t>((rpm_int >> 8) & 0xFF),
            static_cast<uint8_t>((rpm_int >> 0) & 0xFF),
        };
        can_send(id, data, sizeof(data), 2);
        delay(1);
      }
    }

    while (!(FTM0->STATUS & (1 << 7))) {
    }
    {
      const uint32_t start = FTM0->C6V;
      const uint32_t end = FTM0->C7V;
      const int width = (end - start) & 0xFFFF;

      {
        float rpm = static_cast<float>(::std::min(0, width - kMid)) /
                    static_cast<float>(kRange) * kMaxRpm;
        if (flip) {
          rpm *= -1.0f;
        }
        const int32_t rpm_int = rpm;
        uint32_t id = CAN_EFF_FLAG;
        id |= 2;
        id |= (0x03 /* SET_RPM */) << 8;
        uint8_t data[4] = {
            static_cast<uint8_t>((rpm_int >> 24) & 0xFF),
            static_cast<uint8_t>((rpm_int >> 16) & 0xFF),
            static_cast<uint8_t>((rpm_int >> 8) & 0xFF),
            static_cast<uint8_t>((rpm_int >> 0) & 0xFF),
        };
        can_send(id, data, sizeof(data), 2);
        delay(1);
      }

      {
        float rpm = static_cast<float>(::std::max(0, width - kMid)) /
                    static_cast<float>(kRange) * kMaxRpm;
        if (flip) {
          rpm *= -1.0f;
        }
        const int32_t rpm_int = rpm;
        uint32_t id = CAN_EFF_FLAG;
        id |= 3;
        id |= (0x03 /* SET_RPM */) << 8;
        uint8_t data[4] = {
            static_cast<uint8_t>((rpm_int >> 24) & 0xFF),
            static_cast<uint8_t>((rpm_int >> 16) & 0xFF),
            static_cast<uint8_t>((rpm_int >> 8) & 0xFF),
            static_cast<uint8_t>((rpm_int >> 0) & 0xFF),
        };
        can_send(id, data, sizeof(data), 2);
        delay(1);
      }
    }

    while (!(FTM0->STATUS & (1 << 3))) {
    }
    {
      const uint32_t start = FTM0->C2V;
      const uint32_t end = FTM0->C3V;
      const int width = (end - start) & 0xFFFF;

      {
        float rpm = static_cast<float>(::std::min(0, width - kMid)) /
                    static_cast<float>(kRange) * kMaxRpm;
        if (flip) {
          rpm *= -1.0f;
        }
        const int32_t rpm_int = rpm;
        uint32_t id = CAN_EFF_FLAG;
        id |= 4;
        id |= (0x03 /* SET_RPM */) << 8;
        uint8_t data[4] = {
            static_cast<uint8_t>((rpm_int >> 24) & 0xFF),
            static_cast<uint8_t>((rpm_int >> 16) & 0xFF),
            static_cast<uint8_t>((rpm_int >> 8) & 0xFF),
            static_cast<uint8_t>((rpm_int >> 0) & 0xFF),
        };
        can_send(id, data, sizeof(data), 2);
        delay(1);
      }

      {
        float rpm = static_cast<float>(::std::max(0, width - kMid)) /
                    static_cast<float>(kRange) * kMaxRpm;
        if (flip) {
          rpm *= -1.0f;
        }
        const int32_t rpm_int = rpm;
        uint32_t id = CAN_EFF_FLAG;
        id |= 5;
        id |= (0x03 /* SET_RPM */) << 8;
        uint8_t data[4] = {
            static_cast<uint8_t>((rpm_int >> 24) & 0xFF),
            static_cast<uint8_t>((rpm_int >> 16) & 0xFF),
            static_cast<uint8_t>((rpm_int >> 8) & 0xFF),
            static_cast<uint8_t>((rpm_int >> 0) & 0xFF),
        };
        can_send(id, data, sizeof(data), 2);
        delay(1);
      }
    }
  }
}

void SetupPwmFtm(BigFTM *ftm) {
  ftm->MODE = FTM_MODE_WPDIS;
  ftm->MODE = FTM_MODE_WPDIS | FTM_MODE_FTMEN;
  ftm->SC = FTM_SC_CLKS(0) /* Disable counting for now */;

  // Can't change MOD according to the reference manual ("The Dual Edge Capture
  // mode must be used with ...  the FTM counter in Free running counter.").
  ftm->MOD = 0xFFFF;

  // Capturing rising edge.
  ftm->C0SC = FTM_CSC_MSA | FTM_CSC_ELSA;
  // Capturing falling edge.
  ftm->C1SC = FTM_CSC_MSA | FTM_CSC_ELSB;

  // Capturing rising edge.
  ftm->C2SC = FTM_CSC_MSA | FTM_CSC_ELSA;
  // Capturing falling edge.
  ftm->C3SC = FTM_CSC_MSA | FTM_CSC_ELSB;

  // Capturing rising edge.
  ftm->C4SC = FTM_CSC_MSA | FTM_CSC_ELSA;
  // Capturing falling edge.
  ftm->C5SC = FTM_CSC_MSA | FTM_CSC_ELSB;

  // Capturing rising edge.
  ftm->C6SC = FTM_CSC_MSA | FTM_CSC_ELSA;
  // Capturing falling edge.
  ftm->C7SC = FTM_CSC_MSA | FTM_CSC_ELSB;

  ftm->STATUS = 0x00;

  ftm->COMBINE = FTM_COMBINE_DECAP3 | FTM_COMBINE_DECAPEN3 |
                 FTM_COMBINE_DECAP2 | FTM_COMBINE_DECAPEN2 |
                 FTM_COMBINE_DECAP1 | FTM_COMBINE_DECAPEN1 |
                 FTM_COMBINE_DECAP0 | FTM_COMBINE_DECAPEN0;

  // 34.95ms max period before it starts wrapping and being weird.
  ftm->SC = FTM_SC_CLKS(1) /* Use the system clock */ |
            FTM_SC_PS(4) /* Prescaler=32 */;

  ftm->MODE &= ~FTM_MODE_WPDIS;
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

  // Builtin LED.
  PERIPHERAL_BITBAND(GPIOC_PDOR, 5) = 1;
  PORTC_PCR5 = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(1);
  PERIPHERAL_BITBAND(GPIOC_PDDR, 5) = 1;

  // Set up the CAN pins.
  PORTA_PCR12 = PORT_PCR_DSE | PORT_PCR_MUX(2);
  PORTA_PCR13 = PORT_PCR_DSE | PORT_PCR_MUX(2);

  // PWM_IN0
  // FTM0_CH0
  PORTC_PCR1 = PORT_PCR_MUX(4);

  // PWM_IN1
  // FTM0_CH6
  PORTD_PCR6 = PORT_PCR_MUX(4);

  // PWM_IN2
  // FTM0_CH4
  PORTD_PCR4 = PORT_PCR_MUX(4);

  // PWM_IN3
  // FTM3_CH2
  PORTD_PCR2 = PORT_PCR_MUX(4);

  // PWM_IN4
  // FTM0_CH2
  PORTC_PCR3 = PORT_PCR_MUX(4);

  delay(100);

  teensy::UsbDevice usb_device(0, 0x16c0, 0x0492);
  usb_device.SetManufacturer("Seems Reasonable LLC");
  usb_device.SetProduct("Simple Receiver Board");

  teensy::AcmTty tty0(&usb_device);
  global_stdout.store(&tty0, ::std::memory_order_release);
  usb_device.Initialize();

  can_init(0, 1);
  salsa::AdcInitJoystick();
  SetupPwmFtm(FTM0);
  SetupPwmFtm(FTM3);

  // Leave the LEDs on for a bit longer.
  delay(300);
  printf("Done starting up\n");

  // Done starting up, now turn the LED off.
  PERIPHERAL_BITBAND(GPIOC_PDOR, 5) = 0;

  DoReceiverTest2();

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
