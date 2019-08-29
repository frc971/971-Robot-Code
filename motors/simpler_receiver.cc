// This file has the main for the Teensy on the simple receiver board v2 in the
// new robot.

#include <inttypes.h>
#include <stdio.h>
#include <atomic>
#include <chrono>
#include <cmath>

#include "frc971/control_loops/drivetrain/polydrivetrain.h"
#include "motors/core/kinetis.h"
#include "motors/core/time.h"
#include "motors/peripheral/configuration.h"
#include "motors/print/print.h"
#include "motors/seems_reasonable/drivetrain_dog_motor_plant.h"
#include "motors/seems_reasonable/polydrivetrain_dog_motor_plant.h"
#include "motors/util.h"

namespace frc971 {
namespace motors {
namespace {

using ::frc971::constants::ShifterHallEffect;
using ::frc971::control_loops::drivetrain::DrivetrainConfig;
using ::frc971::control_loops::drivetrain::OutputT;
using ::frc971::control_loops::drivetrain::PolyDrivetrain;

namespace chrono = ::std::chrono;

const ShifterHallEffect kThreeStateDriveShifter{0.0, 0.0, 0.25, 0.75};

const DrivetrainConfig<float> &GetDrivetrainConfig() {
  static DrivetrainConfig<float> kDrivetrainConfig{
      ::frc971::control_loops::drivetrain::ShifterType::NO_SHIFTER,
      ::frc971::control_loops::drivetrain::LoopType::OPEN_LOOP,
      ::frc971::control_loops::drivetrain::GyroType::SPARTAN_GYRO,
      ::frc971::control_loops::drivetrain::IMUType::IMU_X,

      ::motors::seems_reasonable::MakeDrivetrainLoop,
      ::motors::seems_reasonable::MakeVelocityDrivetrainLoop,
      ::std::function<StateFeedbackLoop<7, 2, 4, float>()>(),

      chrono::duration_cast<chrono::nanoseconds>(
          chrono::duration<float>(::motors::seems_reasonable::kDt)),
      ::motors::seems_reasonable::kRobotRadius,
      ::motors::seems_reasonable::kWheelRadius, ::motors::seems_reasonable::kV,

      ::motors::seems_reasonable::kHighGearRatio,
      ::motors::seems_reasonable::kLowGearRatio, ::motors::seems_reasonable::kJ,
      ::motors::seems_reasonable::kMass, kThreeStateDriveShifter,
      kThreeStateDriveShifter, true /* default_high_gear */,
      0 /* down_offset */, 0.8 /* wheel_non_linearity */,
      1.2 /* quickturn_wheel_multiplier */, 1.5 /* wheel_multiplier */,
  };

  return kDrivetrainConfig;
};


::std::atomic<PolyDrivetrain<float> *> global_polydrivetrain{nullptr};

// Last width we received on each channel.
uint16_t pwm_input_widths[6];
// When we received a pulse on each channel in milliseconds.
uint32_t pwm_input_times[6];

constexpr int kChannelTimeout = 100;

bool lost_channel(int channel) {
  DisableInterrupts disable_interrupts;
  if (time_after(millis(),
                 time_add(pwm_input_times[channel], kChannelTimeout))) {
    return true;
  }
  return false;
}

// Returns the most recently captured value for the specified input channel
// scaled from -1 to 1, or 0 if it was captured over 100ms ago.
float convert_input_width(int channel) {
  uint16_t width;
  {
    DisableInterrupts disable_interrupts;
    if (time_after(millis(),
                   time_add(pwm_input_times[channel], kChannelTimeout))) {
      return 0;
    }

    width = pwm_input_widths[channel];
  }

  // Values measured with a channel mapped to a button.
  static constexpr uint16_t kMinWidth = 4133;
  static constexpr uint16_t kMaxWidth = 7177;
  if (width < kMinWidth) {
    width = kMinWidth;
  } else if (width > kMaxWidth) {
    width = kMaxWidth;
  }
  return (static_cast<float>(2 * (width - kMinWidth)) /
          static_cast<float>(kMaxWidth - kMinWidth)) -
         1.0f;
}

void SetupPwmInFtm(BigFTM *ftm) {
  ftm->MODE = FTM_MODE_WPDIS;
  ftm->MODE = FTM_MODE_WPDIS | FTM_MODE_FTMEN;
  ftm->SC = FTM_SC_CLKS(0) /* Disable counting for now */;

  // Can't change MOD according to the reference manual ("The Dual Edge Capture
  // mode must be used with ... the FTM counter in Free running counter.").
  ftm->MOD = 0xFFFF;

  // Capturing rising edge.
  ftm->C0SC = FTM_CSC_MSA | FTM_CSC_ELSA;
  // Capturing falling edge.
  ftm->C1SC = FTM_CSC_CHIE | FTM_CSC_MSA | FTM_CSC_ELSB;

  // Capturing rising edge.
  ftm->C2SC = FTM_CSC_MSA | FTM_CSC_ELSA;
  // Capturing falling edge.
  ftm->C3SC = FTM_CSC_CHIE | FTM_CSC_MSA | FTM_CSC_ELSB;

  // Capturing rising edge.
  ftm->C4SC = FTM_CSC_MSA | FTM_CSC_ELSA;
  // Capturing falling edge.
  ftm->C5SC = FTM_CSC_CHIE | FTM_CSC_MSA | FTM_CSC_ELSB;

  // Capturing rising edge.
  ftm->C6SC = FTM_CSC_MSA | FTM_CSC_ELSA;
  // Capturing falling edge.
  ftm->C7SC = FTM_CSC_CHIE | FTM_CSC_MSA | FTM_CSC_ELSB;

  (void)ftm->STATUS;
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

constexpr int kOutputCounts = 37500;
constexpr int kOutputPrescalerShift = 5;

void SetOutputWidth(int output, float ms) {
  static constexpr float kScale = static_cast<float>(
      static_cast<double>(kOutputCounts) / 20.0 /* milliseconds per period */);
  const int width = static_cast<int>(ms * kScale + 0.5f);
  switch (output) {
    case 0:
      FTM3->C0V = width - 1;
      break;
    case 1:
      FTM3->C2V = width - 1;
      break;
    case 2:
      FTM3->C3V = width - 1;
      break;
    case 3:
      FTM3->C1V = width - 1;
      break;
  }
  FTM3->PWMLOAD = FTM_PWMLOAD_LDOK;
}

// 1ms - 2ms (default for a VESC).
void SetupPwmOutFtm(BigFTM *const pwm_ftm) {
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

  pwm_ftm->SC = FTM_SC_TOIE /* Interrupt on overflow */ |
                FTM_SC_CLKS(1) /* Use the system clock */ |
                FTM_SC_PS(kOutputPrescalerShift);
  pwm_ftm->MODE &= ~FTM_MODE_WPDIS;
}

extern "C" void ftm0_isr() {
  while (true) {
    const uint32_t status = FTM0->STATUS;
    if (status == 0) {
      return;
    }

    if (status & (1 << 1)) {
      const uint32_t start = FTM0->C0V;
      const uint32_t end = FTM0->C1V;
      pwm_input_widths[1] = (end - start) & 0xFFFF;
      pwm_input_times[1] = millis();
    }
    if (status & (1 << 5)) {
      const uint32_t start = FTM0->C4V;
      const uint32_t end = FTM0->C5V;
      pwm_input_widths[3] = (end - start) & 0xFFFF;
      pwm_input_times[3] = millis();
    }
    if (status & (1 << 3)) {
      const uint32_t start = FTM0->C2V;
      const uint32_t end = FTM0->C3V;
      pwm_input_widths[4] = (end - start) & 0xFFFF;
      pwm_input_times[4] = millis();
    }

    FTM0->STATUS = 0;
  }
}

extern "C" void ftm3_isr() {
  while (true) {
    const uint32_t status = FTM3->STATUS;
    if (status == 0) {
      return;
    }

    FTM3->STATUS = 0;
  }
}

extern "C" void pit3_isr() {
  PIT_TFLG3 = 1;
  PolyDrivetrain<float> *polydrivetrain =
      global_polydrivetrain.load(::std::memory_order_acquire);

  const bool lost_drive_channel = lost_channel(3) || lost_channel(1);

  if (false) {
    static int count = 0;
    if (++count == 50) {
      count = 0;
      printf("0: %d 1: %d\n", (int)pwm_input_widths[3],
             (int)pwm_input_widths[1]);
    }
  }

  if (polydrivetrain != nullptr) {
    float throttle;
    float wheel;
    if (lost_drive_channel) {
      throttle = 0.0f;
      wheel = 0.0f;
    } else {
      throttle = convert_input_width(1);
      wheel = -convert_input_width(3);
    }
    const bool quickturn = ::std::abs(polydrivetrain->velocity()) < 0.25f;

    if (false) {
      static int count = 0;
      if (++count == 50) {
        count = 0;
        printf("throttle: %d wheel: %d\n", (int)(throttle * 100),
               (int)(wheel * 100));
      }
    }

    OutputT output;

    polydrivetrain->SetGoal(wheel, throttle, quickturn, false);
    polydrivetrain->Update(12.0f);
    polydrivetrain->SetOutput(&output);

    if (false) {
      static int count = 0;
      if (++count == 50) {
        count = 0;
        printf("l: %d r: %d\n", (int)(output.left_voltage * 100),
               (int)(output.right_voltage * 100));
      }
    }

    SetOutputWidth(0, 1.5f - output.left_voltage / 12.0f * 0.5f);
    SetOutputWidth(1, 1.5f + output.right_voltage / 12.0f * 0.5f);
  }
}

}  // namespace

extern "C" {

void *__stack_chk_guard = (void *)0x67111971;
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
  NVIC_SET_SANE_PRIORITY(IRQ_FTM0, 0xa);
  NVIC_SET_SANE_PRIORITY(IRQ_PIT_CH3, 0x5);

  // Builtin LED.
  PERIPHERAL_BITBAND(GPIOC_PDOR, 5) = 1;
  PORTC_PCR5 = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(1);
  PERIPHERAL_BITBAND(GPIOC_PDDR, 5) = 1;

  // PWM_OUT0
  // FTM3_CH0
  PORTD_PCR0 = PORT_PCR_DSE | PORT_PCR_MUX(4);

  // PWM_OUT1
  // FTM3_CH2
  PORTD_PCR2 = PORT_PCR_DSE | PORT_PCR_MUX(4);

  // PWM_OUT2
  // FTM3_CH3
  PORTD_PCR3 = PORT_PCR_DSE | PORT_PCR_MUX(4);

  // PWM_OUT3
  // FTM3_CH1
  PORTD_PCR1 = PORT_PCR_DSE | PORT_PCR_MUX(4);

  // PWM_IN0
  // FTM0_CH1 (doesn't work)
  // PORTC_PCR2 = PORT_PCR_MUX(4);

  // PWM_IN1
  // FTM0_CH0
  PORTC_PCR1 = PORT_PCR_MUX(4);

  // PWM_IN2
  // FTM0_CH5 (doesn't work)
  // PORTD_PCR5 = PORT_PCR_MUX(4);

  // PWM_IN3
  // FTM0_CH4
  PORTD_PCR4 = PORT_PCR_MUX(4);

  // PWM_IN4
  // FTM0_CH2
  PORTC_PCR3 = PORT_PCR_MUX(4);

  // PWM_IN5
  // FTM0_CH3 (doesn't work)
  // PORTC_PCR4 = PORT_PCR_MUX(4);

  delay(100);

  PrintingParameters printing_parameters;
  printing_parameters.dedicated_usb = true;
  const ::std::unique_ptr<PrintingImplementation> printing =
      CreatePrinting(printing_parameters);
  printing->Initialize();

  SIM_SCGC6 |= SIM_SCGC6_PIT;
  // Workaround for errata e7914.
  (void)PIT_MCR;
  PIT_MCR = 0;
  PIT_LDVAL3 = (BUS_CLOCK_FREQUENCY / 200) - 1;
  PIT_TCTRL3 = PIT_TCTRL_TIE | PIT_TCTRL_TEN;

  SetupPwmInFtm(FTM0);
  SetupPwmOutFtm(FTM3);

  PolyDrivetrain<float> polydrivetrain(GetDrivetrainConfig(), nullptr);
  global_polydrivetrain.store(&polydrivetrain, ::std::memory_order_release);

  // Leave the LED on for a bit longer.
  delay(300);
  printf("Done starting up\n");

  // Done starting up, now turn the LED off.
  PERIPHERAL_BITBAND(GPIOC_PDOR, 5) = 0;

  NVIC_ENABLE_IRQ(IRQ_FTM0);
  NVIC_ENABLE_IRQ(IRQ_PIT_CH3);
  printf("Done starting up2\n");

  while (true) {
  }

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
