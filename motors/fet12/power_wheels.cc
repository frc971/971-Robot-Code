#include "motors/core/kinetis.h"

#include <inttypes.h>
#include <math.h>
#include <stdio.h>

#include <atomic>

#include "motors/core/time.h"
#include "motors/peripheral/adc.h"
#include "motors/usb/cdc.h"
#include "motors/usb/usb.h"
#include "motors/util.h"

namespace frc971 {
namespace motors {
namespace {

struct Fet12AdcReadings {
  // 1100 off, 3160 floored (without divider??)
  uint16_t throttle;
};

void AdcInitFet12() {
  AdcInitCommon();

  // EI2C_SCL (end pin) ADC0_SE13
  PORTB_PCR3 = PORT_PCR_MUX(0);
}

Fet12AdcReadings AdcReadFet12(const DisableInterrupts &) {
  Fet12AdcReadings r;

  ADC0_SC1A = 13;
  while (!(ADC0_SC1A & ADC_SC1_COCO)) {
  }
  r.throttle = ADC0_RA;

  return r;
}

bool ReadButton() { return PERIPHERAL_BITBAND(GPIOB_PDIR, 2); }

::std::atomic<teensy::AcmTty *> global_stdout{nullptr};

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

int _write(int /*file*/, char *ptr, int len) {
  teensy::AcmTty *const tty = global_stdout.load(::std::memory_order_acquire);
  if (tty != nullptr) {
    return tty->Write(ptr, len);
  }
  return 0;
}

void __stack_chk_fail(void);

extern char *__brkval;
extern uint32_t __bss_ram_start__[];
extern uint32_t __heap_start__[];
extern uint32_t __stack_end__[];

}  // extern "C"

constexpr int kOutputCounts = 37500;
constexpr int kOutputPrescalerShift = 4;

void SetOutputWidth(float ms) {
  static constexpr float kScale = static_cast<float>(
      static_cast<double>(kOutputCounts) / 10.0 /* milliseconds per period */);
  const int width = static_cast<int>(ms * kScale + 0.5f);
  FTM3->C6V = width - 1;
  FTM3->PWMLOAD = FTM_PWMLOAD_LDOK;
}

}  // namespace

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
  NVIC_SET_SANE_PRIORITY(IRQ_FTM0, 0x3);

  // Set the LED's pin to output mode.
  PERIPHERAL_BITBAND(GPIOC_PDDR, 5) = 1;
  PORTC_PCR5 = PORT_PCR_DSE | PORT_PCR_MUX(1);

  // EI2C_SCL (not end) PTB3
  PORTB_PCR2 = PORT_PCR_MUX(1);

#if 0
  PERIPHERAL_BITBAND(GPIOA_PDDR, 15) = 1;
  PORTA_PCR15 = PORT_PCR_DSE | PORT_PCR_MUX(1);
#endif

  DMA.CR = M_DMA_EMLM;

  teensy::UsbDevice usb_device(0, 0x16c0, 0x0490);
  usb_device.SetManufacturer("FRC 971 Spartan Robotics");
  usb_device.SetProduct("FET12 power wheels mode");
  teensy::AcmTty tty1(&usb_device);
  teensy::AcmTty tty2(&usb_device);
  global_stdout.store(&tty1, ::std::memory_order_release);
  usb_device.Initialize();

  AdcInitFet12();
  delay(1000);

#if 0
  GPIOD_PCOR = 1 << 3;
  PERIPHERAL_BITBAND(GPIOD_PDDR, 3) = 1;
  PORTD_PCR3 = PORT_PCR_DSE | PORT_PCR_MUX(1);
  delay(1000);
  GPIOD_PSOR = 1 << 3;
  delay(1000);
  GPIOD_PCOR = 1 << 3;
  delay(1000);
#endif

  delay(1000);

  // Index pin
  PORTA_PCR7 = PORT_PCR_MUX(1);
  // FTM1_QD_PH{A,B}
  PORTB_PCR0 = PORT_PCR_MUX(6);
  PORTB_PCR1 = PORT_PCR_MUX(6);

  // FTM3_CH6 for PWM_IN (used as output)
  PORTE_PCR11 = PORT_PCR_MUX(6);

  auto *const encoder_ftm = FTM1;
  // PWMSYNC doesn't matter because we set SYNCMODE down below.
  encoder_ftm->MODE = FTM_MODE_WPDIS;
  encoder_ftm->MODE = FTM_MODE_WPDIS | FTM_MODE_FTMEN;
  encoder_ftm->SC =
      FTM_SC_CLKS(1) /* Use the system clock (not sure it matters) */ |
      FTM_SC_PS(0) /* Don't prescale the clock (not sure it matters) */;

  encoder_ftm->MOD = 4095;

  // I think you have to set this to something other than 0 for the quadrature
  // encoder mode to actually work? This is "input capture on rising edge only",
  // which should be fine.
  encoder_ftm->C0SC = FTM_CSC_ELSA;
  encoder_ftm->C1SC = FTM_CSC_ELSA;

  encoder_ftm->FILTER = FTM_FILTER_CH0FVAL(0) /* No filter */ |
                        FTM_FILTER_CH1FVAL(0) /* No filter */;

  // Could set PHAFLTREN and PHBFLTREN here to enable the filters.
  encoder_ftm->QDCTRL = FTM_QDCTRL_QUADEN;

  encoder_ftm->SYNCONF =
      FTM_SYNCONF_SWWRBUF /* Software trigger flushes MOD */ |
      FTM_SYNCONF_SWRSTCNT /* Software trigger resets the count */ |
      FTM_SYNCONF_SYNCMODE /* Use the new synchronization mode */;

  encoder_ftm->SYNC = FTM_SYNC_SWSYNC /* Flush everything out right now */;
  // Wait for the software synchronization to finish.
  while (encoder_ftm->SYNC & FTM_SYNC_SWSYNC) {
  }

  auto *const pwm_ftm = FTM3;
  // PWMSYNC doesn't matter because we set SYNCMODE down below.
  pwm_ftm->MODE = FTM_MODE_WPDIS;
  pwm_ftm->MODE = FTM_MODE_WPDIS | FTM_MODE_FTMEN;
  pwm_ftm->SC = FTM_SC_CLKS(0) /* Disable counting for now */ |
                FTM_SC_PS(kOutputPrescalerShift);

  pwm_ftm->CNTIN = 0;
  pwm_ftm->CNT = 0;
  pwm_ftm->MOD = kOutputCounts - 1;

  // High-true edge-aligned mode (turns on at start, off at match).
  pwm_ftm->C0SC = FTM_CSC_MSB | FTM_CSC_ELSB;
  pwm_ftm->C1SC = FTM_CSC_MSB | FTM_CSC_ELSB;
  pwm_ftm->C2SC = FTM_CSC_MSB | FTM_CSC_ELSB;
  pwm_ftm->C3SC = FTM_CSC_MSB | FTM_CSC_ELSB;
  pwm_ftm->C4SC = FTM_CSC_MSB | FTM_CSC_ELSB;
  pwm_ftm->C5SC = FTM_CSC_MSB | FTM_CSC_ELSB;
  pwm_ftm->C6SC = FTM_CSC_MSB | FTM_CSC_ELSB;
  pwm_ftm->C7SC = FTM_CSC_MSB | FTM_CSC_ELSB;

  pwm_ftm->COMBINE = FTM_COMBINE_SYNCEN3 /* Synchronize updates usefully */ |
                     FTM_COMBINE_SYNCEN2 /* Synchronize updates usefully */ |
                     FTM_COMBINE_SYNCEN1 /* Synchronize updates usefully */ |
                     FTM_COMBINE_SYNCEN0 /* Synchronize updates usefully */;

  // Initialize all the channels to 0.
  pwm_ftm->OUTINIT = 0;

  // All of the channels are active high.
  pwm_ftm->POL = 0;

  pwm_ftm->SYNCONF =
      FTM_SYNCONF_HWWRBUF /* Hardware trigger flushes switching points */ |
      FTM_SYNCONF_SWWRBUF /* Software trigger flushes switching points */ |
      FTM_SYNCONF_SWRSTCNT /* Software trigger resets the count */ |
      FTM_SYNCONF_SYNCMODE /* Use the new synchronization mode */;

  // Don't want any intermediate loading points.
  pwm_ftm->PWMLOAD = 0;

  // This has to happen after messing with SYNCONF, and should happen after
  // messing with various other things so the values can get flushed out of the
  // buffers.
  pwm_ftm->SYNC = FTM_SYNC_SWSYNC /* Flush everything out right now */ |
                  FTM_SYNC_CNTMAX /* Load new values at the end of the cycle */;
  // Wait for the software synchronization to finish.
  while (pwm_ftm->SYNC & FTM_SYNC_SWSYNC) {
  }

  // Don't let any memory accesses sneak past here, because we actually
  // need everything to be starting up.
  __asm__("" :: : "memory");

  // Give everything a chance to get going.
  delay(100);

  printf("Ram start:   %p\n", __bss_ram_start__);
  printf("Heap start:  %p\n", __heap_start__);
  printf("Heap end:    %p\n", __brkval);
  printf("Stack start: %p\n", __stack_end__);

  encoder_ftm->MODE &= ~FTM_MODE_WPDIS;
  pwm_ftm->SC = FTM_SC_TOIE /* Interrupt on overflow */ |
                FTM_SC_CLKS(1) /* Use the system clock */ |
                FTM_SC_PS(kOutputPrescalerShift);
  pwm_ftm->MODE &= ~FTM_MODE_WPDIS;

  GPIOC_PSOR = 1 << 5;

  uint16_t old_encoder = FTM1->CNT;
  uint32_t start_time = micros();
  while (true) {
    const uint32_t end_time = start_time + UINT32_C(500);
    while (micros() < end_time) {
    }
    start_time = end_time;

    Fet12AdcReadings adc_readings;
    {
      DisableInterrupts disable_interrupts;
      adc_readings = AdcReadFet12(disable_interrupts);
    }
    static constexpr int kThrottleMin = 700;
    static constexpr int kThrottleMax = 2000;
    //static constexpr int kThrottleMin = 1100;
    //static constexpr int kThrottleMax = 3190;
    const float pedal_position = ::std::min(
        1.0f,
        ::std::max(0.0f,
                   static_cast<float>(adc_readings.throttle - kThrottleMin) /
                       static_cast<float>(kThrottleMax - kThrottleMin)));

    const uint16_t new_encoder = FTM1->CNT;
    // Full speed is ~418.
    // Low gear is positive.
    int16_t encoder_delta =
        static_cast<int16_t>(new_encoder) - static_cast<int16_t>(old_encoder);
    if (encoder_delta < -2048) {
      encoder_delta += 4096;
    }
    if (encoder_delta > 2048) {
      encoder_delta -= 4096;
    }
    old_encoder = new_encoder;

    // Positive -> low gear
    float speed = ::std::min(
        1.0f,
        ::std::max(-1.0f, static_cast<float>(encoder_delta) / 418.0f / 2.0f));

    float out_command = pedal_position;

    static constexpr float kMaxCurrentFull = 0.14f;
    static constexpr float kMaxCurrentStopped = 0.27f;
    float abs_speed;
    if (speed > 0.0f) {
      abs_speed = speed;
    } else {
      abs_speed = -speed;
    }
    float max_current =
        abs_speed * (kMaxCurrentFull - kMaxCurrentStopped) + kMaxCurrentStopped;
    if (abs_speed < 0.042f) {
      max_current = 0.4f;
    }
    if (speed > 0.0f) {
      out_command =
          ::std::min(speed + max_current,
                     ::std::max(speed - 2.0f * max_current, out_command));
    } else {
      out_command = ::std::min(speed + 2.0f * max_current,
                               ::std::max(speed - max_current, out_command));
    }
    if (speed > 0.04f) {
      // Force some command as long as we're rolling to avoid engaging motor
      // braking.
      out_command = ::std::max(0.10f, out_command);
    }

    static float slew_limited_command = 0.0f;
    constexpr float kMaxChangePerCycle = 1.0f / 150.0f;

    if (out_command < slew_limited_command - kMaxChangePerCycle) {
      out_command = slew_limited_command - kMaxChangePerCycle;
    } else if (out_command > slew_limited_command + kMaxChangePerCycle) {
      out_command = slew_limited_command + kMaxChangePerCycle;
    }

    slew_limited_command = out_command;

    const float pwm_out = 1.5f + -slew_limited_command / 2.0f;
    SetOutputWidth(pwm_out);

    static int i = 0;
    if (i == 500) {
      i = 0;
      printf("enc %" PRIu32 " throttle %" PRIu16 " %d out %d %d %d\n",
             FTM1->CNT, adc_readings.throttle, ReadButton(),
             (int)(pwm_out * 1000), (int)encoder_delta,
             (int)(abs_speed * 1000));
    }
    ++i;
  }

  return 0;
}

}  // namespace motors
}  // namespace frc971
