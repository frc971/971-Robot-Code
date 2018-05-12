#include "motors/core/kinetis.h"

#include <inttypes.h>
#include <stdio.h>

#include <atomic>
#include <cmath>

#include "frc971/control_loops/drivetrain/integral_haptic_trigger.h"
#include "frc971/control_loops/drivetrain/integral_haptic_wheel.h"
#include "motors/core/time.h"
#include "motors/motor.h"
#include "motors/peripheral/adc.h"
#include "motors/peripheral/can.h"
#include "motors/pistol_grip/motor_controls.h"
#include "motors/usb/cdc.h"
#include "motors/usb/usb.h"
#include "motors/util.h"

#define MOTOR0_PWM_FTM FTM3
#define MOTOR0_ENCODER_FTM FTM2
#define MOTOR1_PWM_FTM FTM0
#define MOTOR1_ENCODER_FTM FTM1

extern const float kWheelCoggingTorque[4096];
extern const float kTriggerCoggingTorque[4096];

namespace frc971 {
namespace motors {
namespace {

using ::frc971::control_loops::drivetrain::MakeIntegralHapticTriggerPlant;
using ::frc971::control_loops::drivetrain::MakeIntegralHapticTriggerObserver;
using ::frc971::control_loops::drivetrain::MakeIntegralHapticWheelPlant;
using ::frc971::control_loops::drivetrain::MakeIntegralHapticWheelObserver;

constexpr float kHapticWheelCurrentLimit = static_cast<float>(
    ::frc971::control_loops::drivetrain::kHapticWheelCurrentLimit);
constexpr float kHapticTriggerCurrentLimit = static_cast<float>(
    ::frc971::control_loops::drivetrain::kHapticTriggerCurrentLimit);

::std::atomic<Motor *> global_motor0{nullptr}, global_motor1{nullptr};
::std::atomic<teensy::AcmTty *> global_stdout{nullptr};

// Angle last time the current loop ran.
::std::atomic<float> global_wheel_angle{0.0f};
::std::atomic<float> global_trigger_angle{0.0f};

// Wheel observer/plant.
::std::atomic<StateFeedbackObserver<3, 1, 1, float> *> global_wheel_observer{
    nullptr};
::std::atomic<StateFeedbackPlant<3, 1, 1, float> *> global_wheel_plant{nullptr};
// Throttle observer/plant.
::std::atomic<StateFeedbackObserver<3, 1, 1, float> *> global_trigger_observer{
    nullptr};
::std::atomic<StateFeedbackPlant<3, 1, 1, float> *> global_trigger_plant{
    nullptr};

// Torques for the current loop to apply.
::std::atomic<float> global_wheel_current{0.0f};
::std::atomic<float> global_trigger_torque{0.0f};

constexpr int kSwitchingDivisor = 2;

float analog_ratio(uint16_t reading) {
  static constexpr uint16_t kMin = 260, kMax = 3812;
  return static_cast<float>(::std::max(::std::min(reading, kMax), kMin) -
                            kMin) /
         static_cast<float>(kMax - kMin);
}

constexpr float InterpolateFloat(float x1, float x0, float y1, float y0, float x) {
  return (x - x0) * (y1 - y0) / (x1 - x0) + y0;
}

float absolute_wheel(float wheel_position) {
  if (wheel_position < 0.43f) {
    wheel_position += 1.0f;
  }
  wheel_position -= 0.462f + 0.473f;
  return wheel_position;
}

extern "C" {

void *__stack_chk_guard = (void *)0x67111971;
void __stack_chk_fail() {
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

extern uint32_t __bss_ram_start__[], __bss_ram_end__[];
extern uint32_t __data_ram_start__[], __data_ram_end__[];
extern uint32_t __heap_start__[], __heap_end__[];
extern uint32_t __stack_end__[];

}  // extern "C"

constexpr float kWheelMaxExtension = 1.0f;
constexpr float kWheelFrictionMax = 0.2f;
float WheelCenteringCurrent(float scalar, float angle, float velocity) {
  float friction_goal_current = -angle * 10.0f;
  if (friction_goal_current > kWheelFrictionMax) {
    friction_goal_current = kWheelFrictionMax;
  } else if (friction_goal_current < -kWheelFrictionMax) {
    friction_goal_current = -kWheelFrictionMax;
  }

  constexpr float kWheelSpringNonlinearity = 0.45f;

  float goal_current = -((1.0f - kWheelSpringNonlinearity) * angle +
                         kWheelSpringNonlinearity * angle * angle * angle) *
                           6.0f -
                       velocity * 0.04f;
  if (goal_current > 5.0f - scalar) {
    goal_current = 5.0f - scalar;
  } else if (goal_current < -5.0f + scalar) {
    goal_current = -5.0f + scalar;
  }

  return goal_current * scalar + friction_goal_current;
}

extern "C" void ftm0_isr() {
  SmallAdcReadings readings;
  {
    DisableInterrupts disable_interrupts;
    readings = AdcReadSmall1(disable_interrupts);
  }
  uint32_t encoder =
      global_motor1.load(::std::memory_order_relaxed)->wrapped_encoder();
  int32_t absolute_encoder = global_motor1.load(::std::memory_order_relaxed)
                                 ->absolute_encoder(encoder);

  const float angle = absolute_encoder / static_cast<float>((15320 - 1488) / 2);
  global_wheel_angle.store(angle);

  float goal_current = -global_wheel_current.load(::std::memory_order_relaxed) +
                       kWheelCoggingTorque[encoder];

  global_motor1.load(::std::memory_order_relaxed)->SetGoalCurrent(goal_current);
  global_motor1.load(::std::memory_order_relaxed)
      ->HandleInterrupt(BalanceSimpleReadings(readings.currents), encoder);
}

constexpr float kTriggerMaxExtension = -0.70f;
constexpr float kTriggerCenter = 0.0f;
constexpr float kCenteringStiffness = 0.15f;
float TriggerCenteringCurrent(float trigger_angle) {
  float goal_current = (kTriggerCenter - trigger_angle) * 3.0f;
  float knotch_goal_current = (kTriggerCenter - trigger_angle) * 8.0f;
  if (knotch_goal_current < -kCenteringStiffness) {
    knotch_goal_current = -kCenteringStiffness;
  } else if (knotch_goal_current > kCenteringStiffness) {
    knotch_goal_current = kCenteringStiffness;
  }

  goal_current += knotch_goal_current;

  if (goal_current < -1.0f) {
    goal_current = -1.0f;
  } else if (goal_current > 1.0f) {
    goal_current = 1.0f;
    if (trigger_angle < kTriggerMaxExtension) {
      goal_current -= (30.0f * (trigger_angle - kTriggerMaxExtension));
      if (goal_current > 4.0f) {
        goal_current = 4.0f;
      }
    }
  }
  return goal_current;
}

extern "C" void ftm3_isr() {
  SmallAdcReadings readings;
  {
    DisableInterrupts disable_interrupts;
    readings = AdcReadSmall0(disable_interrupts);
  }
  uint32_t encoder =
      global_motor0.load(::std::memory_order_relaxed)->wrapped_encoder();
  int32_t absolute_encoder = global_motor0.load(::std::memory_order_relaxed)
                                 ->absolute_encoder(encoder);

  float trigger_angle = absolute_encoder / 1370.f;

  const float goal_current =
      -global_trigger_torque.load(::std::memory_order_relaxed) +
      kTriggerCoggingTorque[encoder];

  global_motor0.load(::std::memory_order_relaxed)->SetGoalCurrent(goal_current);
  global_motor0.load(::std::memory_order_relaxed)
      ->HandleInterrupt(BalanceSimpleReadings(readings.currents), encoder);

  global_trigger_angle.store(trigger_angle);
}

int ConvertFloat16(float val) {
  int result = static_cast<int>(val * 32768.0f) + 32768;
  if (result > 0xffff) {
    result = 0xffff;
  } else if (result < 0) {
    result = 0;
  }
  return result;
}
int ConvertFloat14(float val) {
  int result = static_cast<int>(val * 8192.0f) + 8192;
  if (result > 0x3fff) {
    result = 0x3fff;
  } else if (result < 0) {
    result = 0;
  }
  return result;
}

extern "C" void pit3_isr() {
  PIT_TFLG3 = 1;
  const float absolute_trigger_angle =
      global_trigger_angle.load(::std::memory_order_relaxed);
  const float absolute_wheel_angle =
      global_wheel_angle.load(::std::memory_order_relaxed);

  // Force a barrier here so we sample everything guaranteed at the beginning.
  __asm__("" ::: "memory");
  const float absolute_wheel_angle_radians =
      absolute_wheel_angle * static_cast<float>(M_PI) * (338.16f / 360.0f);
  const float absolute_trigger_angle_radians =
      absolute_trigger_angle * static_cast<float>(M_PI) * (45.0f / 360.0f);

  static uint32_t last_command_time = 0;
  static float trigger_goal_position = 0.0f;
  static float trigger_goal_velocity = 0.0f;
  static float trigger_haptic_current = 0.0f;
  static bool trigger_centering = true;
  static bool trigger_haptics = false;
  {
    uint8_t data[8];
    int length;
    can_receive(data, &length, 0);
    if (length > 0) {
      last_command_time = micros();
      trigger_goal_position =
          static_cast<float>(
              static_cast<int32_t>(static_cast<uint32_t>(data[0]) |
                                   (static_cast<uint32_t>(data[1]) << 8)) -
              32768) /
          32768.0f * M_PI / 8.0;
      trigger_goal_velocity =
          static_cast<float>(
              static_cast<int32_t>(static_cast<uint32_t>(data[2]) |
                                   (static_cast<uint32_t>(data[3]) << 8)) -
              32768) /
          32768.0f * 4.0f;

      trigger_haptic_current =
          static_cast<float>(
              static_cast<int32_t>(static_cast<uint32_t>(data[4]) |
                                   (static_cast<uint32_t>(data[5]) << 8)) -
              32768) /
          32768.0f * 2.0f;
      if (trigger_haptic_current > kHapticTriggerCurrentLimit) {
        trigger_haptic_current = kHapticTriggerCurrentLimit;
      } else if (trigger_haptic_current < -kHapticTriggerCurrentLimit) {
        trigger_haptic_current = -kHapticTriggerCurrentLimit;
      }
      trigger_centering = !!(data[7] & 0x01);
      trigger_haptics = !!(data[7] & 0x02);
    }
  }

  static float wheel_goal_position = 0.0f;
  static float wheel_goal_velocity = 0.0f;
  static float wheel_haptic_current = 0.0f;
  static float wheel_kp = 0.0f;
  static bool wheel_centering = true;
  static float wheel_centering_scalar = 0.25f;
  {
    uint8_t data[8];
    int length;
    can_receive(data, &length, 1);
    if (length == 8) {
      last_command_time = micros();
      wheel_goal_position =
          static_cast<float>(
              static_cast<int32_t>(static_cast<uint32_t>(data[0]) |
                                   (static_cast<uint32_t>(data[1]) << 8)) -
              32768) /
          32768.0f * M_PI;
      wheel_goal_velocity =
          static_cast<float>(
              static_cast<int32_t>(static_cast<uint32_t>(data[2]) |
                                   (static_cast<uint32_t>(data[3]) << 8)) -
              32768) /
          32768.0f * 10.0f;

      wheel_haptic_current =
          static_cast<float>(
              static_cast<int32_t>(static_cast<uint32_t>(data[4]) |
                                   (static_cast<uint32_t>(data[5]) << 8)) -
              32768) /
          32768.0f * 2.0f;
      if (wheel_haptic_current > kHapticWheelCurrentLimit) {
        wheel_haptic_current = kHapticWheelCurrentLimit;
      } else if (wheel_haptic_current < -kHapticWheelCurrentLimit) {
        wheel_haptic_current = -kHapticWheelCurrentLimit;
      }
      wheel_kp = static_cast<float>(data[6]) * 30.0f / 255.0f;
      wheel_centering = !!(data[7] & 0x01);
      wheel_centering_scalar = ((data[7] >> 1) & 0x7f) / 127.0f;
    }
  }

  static constexpr uint32_t kTimeout = 100000;
  if (!time_after(time_add(last_command_time, kTimeout), micros())) {
    last_command_time = time_subtract(micros(), kTimeout);
    trigger_goal_position = 0.0f;
    trigger_goal_velocity = 0.0f;
    trigger_haptic_current = 0.0f;
    trigger_centering = true;
    trigger_haptics = false;

    wheel_goal_position = 0.0f;
    wheel_goal_velocity = 0.0f;
    wheel_haptic_current = 0.0f;
    wheel_centering = true;
    wheel_centering_scalar = 0.25f;
    // Avoid wrapping back into the valid range.
    last_command_time = time_subtract(micros(), kTimeout);
  }

  StateFeedbackPlant<3, 1, 1, float> *const trigger_plant =
      global_trigger_plant.load(::std::memory_order_relaxed);
  StateFeedbackObserver<3, 1, 1, float> *const trigger_observer =
      global_trigger_observer.load(::std::memory_order_relaxed);
  ::Eigen::Matrix<float, 1, 1> trigger_Y;
  trigger_Y << absolute_trigger_angle_radians;
  trigger_observer->Correct(*trigger_plant,
                            ::Eigen::Matrix<float, 1, 1>::Zero(), trigger_Y);

  StateFeedbackPlant<3, 1, 1, float> *const wheel_plant =
      global_wheel_plant.load(::std::memory_order_relaxed);
  StateFeedbackObserver<3, 1, 1, float> *const wheel_observer =
      global_wheel_observer.load(::std::memory_order_relaxed);
  ::Eigen::Matrix<float, 1, 1> wheel_Y;
  wheel_Y << absolute_wheel_angle_radians;
  wheel_observer->Correct(*wheel_plant, ::Eigen::Matrix<float, 1, 1>::Zero(),
                          wheel_Y);

  float kWheelD = (wheel_kp - 10.0f) * (0.25f - 0.20f) / 5.0f + 0.20f;
  if (wheel_kp < 0.5f) {
    kWheelD = wheel_kp * 0.05f / 0.5f;
  } else if (wheel_kp < 1.0f) {
    kWheelD = InterpolateFloat(1.0f, 0.5f, 0.06f, 0.05f, wheel_kp);
  } else if (wheel_kp < 2.0f) {
    kWheelD = InterpolateFloat(2.0f, 1.0f, 0.08f, 0.06f, wheel_kp);
  } else if (wheel_kp < 3.0f) {
    kWheelD = InterpolateFloat(3.0f, 2.0f, 0.10f, 0.08f, wheel_kp);
  } else if (wheel_kp < 5.0f) {
    kWheelD = InterpolateFloat(5.0f, 3.0f, 0.13f, 0.10f, wheel_kp);
  } else if (wheel_kp < 10.0f) {
    kWheelD = InterpolateFloat(10.0f, 5.0f, 0.20f, 0.13f, wheel_kp);
  }

  float wheel_goal_current = wheel_haptic_current;

  wheel_goal_current +=
      (wheel_goal_position - absolute_wheel_angle_radians) * wheel_kp +
      (wheel_goal_velocity - wheel_observer->X_hat()(1, 0)) * kWheelD;

  // Compute the torques to apply to each motor.
  if (wheel_centering) {
    wheel_goal_current +=
        WheelCenteringCurrent(wheel_centering_scalar, absolute_wheel_angle,
                              wheel_observer->X_hat()(1, 0));
  }

  if (wheel_goal_current > kHapticWheelCurrentLimit) {
    wheel_goal_current = kHapticWheelCurrentLimit;
  } else if (wheel_goal_current < -kHapticWheelCurrentLimit) {
    wheel_goal_current = -kHapticWheelCurrentLimit;
  }
  global_wheel_current.store(wheel_goal_current, ::std::memory_order_relaxed);

  constexpr float kTriggerP =
      static_cast<float>(::frc971::control_loops::drivetrain::kHapticTriggerP);
  constexpr float kTriggerD =
      static_cast<float>(::frc971::control_loops::drivetrain::kHapticTriggerD);
  float trigger_goal_current = trigger_haptic_current;
  if (trigger_haptics) {
    trigger_goal_current +=
        (trigger_goal_position - absolute_trigger_angle_radians) * kTriggerP +
        (trigger_goal_velocity - trigger_observer->X_hat()(1, 0)) * kTriggerD;
  }

  if (trigger_centering) {
    trigger_goal_current += TriggerCenteringCurrent(absolute_trigger_angle);
  }

  if (trigger_goal_current > kHapticTriggerCurrentLimit) {
    trigger_goal_current = kHapticTriggerCurrentLimit;
  } else if (trigger_goal_current < -kHapticTriggerCurrentLimit) {
    trigger_goal_current = -kHapticTriggerCurrentLimit;
  }
  global_trigger_torque.store(trigger_goal_current,
                              ::std::memory_order_relaxed);

  uint8_t buttons = 0;
  if (!PERIPHERAL_BITBAND(GPIOA_PDIR, 14)) {
    buttons |= 0x1;
  }
  if (!PERIPHERAL_BITBAND(GPIOE_PDIR, 26)) {
    buttons |= 0x2;
  }
  if (!PERIPHERAL_BITBAND(GPIOC_PDIR, 7)) {
    buttons |= 0x4;
  }
  if (!PERIPHERAL_BITBAND(GPIOD_PDIR, 0)) {
    buttons |= 0x8;
  }

  float trigger_angle = absolute_trigger_angle;

  // Adjust the trigger range for reporting back.
  // TODO(austin): We'll likely need to make this symmetric for the controls to
  // work out well.
  if (trigger_angle > kTriggerCenter) {
    trigger_angle = (trigger_angle - kTriggerCenter) / (1.0f - kTriggerCenter);
  } else {
    trigger_angle = (trigger_angle - kTriggerCenter) /
                    (kTriggerCenter - kTriggerMaxExtension);
  }

  // TODO(austin): Class + fns.  This is a mess.
  // TODO(austin): Move this to a separate file.  It's too big.
  int can_trigger = ConvertFloat16(absolute_trigger_angle);
  int can_trigger_velocity =
      ConvertFloat16(trigger_observer->X_hat()(1, 0) / 50.0f);
  int can_trigger_torque =
      ConvertFloat16(trigger_observer->X_hat()(2, 0) * 2.0f);
  int can_trigger_current = ConvertFloat14(trigger_goal_current / 10.0f);

  int can_wheel = ConvertFloat16(absolute_wheel_angle);
  int can_wheel_velocity =
      ConvertFloat16(wheel_observer->X_hat()(1, 0) / 50.0f);
  int can_wheel_torque = ConvertFloat16(wheel_observer->X_hat()(2, 0) * 2.0f);
  int can_wheel_current = ConvertFloat14(wheel_goal_current / 10.0f);

  {
    const uint8_t trigger_joystick_values[8] = {
        static_cast<uint8_t>(can_trigger & 0xff),
        static_cast<uint8_t>((can_trigger >> 8) & 0xff),
        static_cast<uint8_t>(can_trigger_velocity & 0xff),
        static_cast<uint8_t>((can_trigger_velocity >> 8) & 0xff),
        static_cast<uint8_t>(can_trigger_torque & 0xff),
        static_cast<uint8_t>((can_trigger_torque >> 8) & 0xff),
        static_cast<uint8_t>(can_trigger_current & 0xff),
        static_cast<uint8_t>(((buttons & 0x3) << 6) |
                             (can_trigger_current >> 8))};
    const uint8_t wheel_joystick_values[8] = {
        static_cast<uint8_t>(can_wheel & 0xff),
        static_cast<uint8_t>((can_wheel >> 8) & 0xff),
        static_cast<uint8_t>(can_wheel_velocity & 0xff),
        static_cast<uint8_t>((can_wheel_velocity >> 8) & 0xff),
        static_cast<uint8_t>(can_wheel_torque & 0xff),
        static_cast<uint8_t>((can_wheel_torque >> 8) & 0xff),
        static_cast<uint8_t>(can_wheel_current & 0xff),
        static_cast<uint8_t>(((buttons & 0xc) << 4) |
                             (can_wheel_current >> 8))};

    can_send(0, trigger_joystick_values, 8, 2);
    can_send(1, wheel_joystick_values, 8, 3);
  }

  ::Eigen::Matrix<float, 1, 1> trigger_U;
  trigger_U << trigger_goal_current;
  ::Eigen::Matrix<float, 1, 1> wheel_U;
  wheel_U << wheel_goal_current;
  trigger_observer->Predict(trigger_plant, trigger_U,
                            ::std::chrono::milliseconds(1));
  wheel_observer->Predict(wheel_plant, wheel_U, ::std::chrono::milliseconds(1));
}

void ConfigurePwmFtm(BigFTM *pwm_ftm) {
  // Put them all into combine active-high mode, and all the low ones staying
  // off all the time by default. We'll then use only the low ones.
  pwm_ftm->C0SC = FTM_CSC_ELSB;
  pwm_ftm->C0V = 0;
  pwm_ftm->C1SC = FTM_CSC_ELSB;
  pwm_ftm->C1V = 0;
  pwm_ftm->C2SC = FTM_CSC_ELSB;
  pwm_ftm->C2V = 0;
  pwm_ftm->C3SC = FTM_CSC_ELSB;
  pwm_ftm->C3V = 0;
  pwm_ftm->C4SC = FTM_CSC_ELSB;
  pwm_ftm->C4V = 0;
  pwm_ftm->C5SC = FTM_CSC_ELSB;
  pwm_ftm->C5V = 0;
  pwm_ftm->C6SC = FTM_CSC_ELSB;
  pwm_ftm->C6V = 0;
  pwm_ftm->C7SC = FTM_CSC_ELSB;
  pwm_ftm->C7V = 0;

  pwm_ftm->COMBINE = FTM_COMBINE_SYNCEN3 /* Synchronize updates usefully */ |
                     FTM_COMBINE_COMP3 /* Make them complementary */ |
                     FTM_COMBINE_COMBINE3 /* Combine the channels */ |
                     FTM_COMBINE_SYNCEN2 /* Synchronize updates usefully */ |
                     FTM_COMBINE_COMP2 /* Make them complementary */ |
                     FTM_COMBINE_COMBINE2 /* Combine the channels */ |
                     FTM_COMBINE_SYNCEN1 /* Synchronize updates usefully */ |
                     FTM_COMBINE_COMP1 /* Make them complementary */ |
                     FTM_COMBINE_COMBINE1 /* Combine the channels */ |
                     FTM_COMBINE_SYNCEN0 /* Synchronize updates usefully */ |
                     FTM_COMBINE_COMP0 /* Make them complementary */ |
                     FTM_COMBINE_COMBINE0 /* Combine the channels */;
}

bool CountValid(uint32_t count) {
  static constexpr int kMaxMovement = 1;
  return count <= kMaxMovement || count >= (4096 - kMaxMovement);
}

bool ZeroMotors(uint16_t *motor0_offset, uint16_t *motor1_offset,
                uint16_t *wheel_offset) {
  static constexpr int kNumberSamples = 1024;
  static_assert(UINT16_MAX * kNumberSamples <= UINT32_MAX, "Too many samples");
  uint32_t motor0_sum = 0, motor1_sum = 0, wheel_sum = 0;

  // First clear both encoders.
  MOTOR0_ENCODER_FTM->CNT = MOTOR1_ENCODER_FTM->CNT = 0;
  for (int i = 0; i < kNumberSamples; ++i) {
    delay(1);

    if (!CountValid(MOTOR0_ENCODER_FTM->CNT)) {
      printf("Motor 0 moved too much\n");
      return false;
    }
    if (!CountValid(MOTOR1_ENCODER_FTM->CNT)) {
      printf("Motor 1 moved too much\n");
      return false;
    }

    DisableInterrupts disable_interrupts;
    const SmallInitReadings readings = AdcReadSmallInit(disable_interrupts);
    motor0_sum += readings.motor0_abs;
    motor1_sum += readings.motor1_abs;
    wheel_sum += readings.wheel_abs;
  }

  *motor0_offset = (motor0_sum + kNumberSamples / 2) / kNumberSamples;
  *motor1_offset = (motor1_sum + kNumberSamples / 2) / kNumberSamples;
  *wheel_offset = (wheel_sum + kNumberSamples / 2) / kNumberSamples;

  return true;
}

}  // namespace

extern "C" int main() {
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
  NVIC_SET_SANE_PRIORITY(IRQ_FTM3, 0x3);
  NVIC_SET_SANE_PRIORITY(IRQ_PIT_CH3, 0x5);

  // Set the LED's pin to output mode.
  PERIPHERAL_BITBAND(GPIOC_PDDR, 5) = 1;
  PORTC_PCR5 = PORT_PCR_DSE | PORT_PCR_MUX(1);

  // Set up the CAN pins.
  PORTA_PCR12 = PORT_PCR_DSE | PORT_PCR_MUX(2);
  PORTA_PCR13 = PORT_PCR_DSE | PORT_PCR_MUX(2);

  // .1ms filter time.
  PORTA_DFWR = PORTC_DFWR = PORTD_DFWR = PORTE_DFWR = 6000;

  // BTN0
  PORTC_PCR7 = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_MUX(1);
  PORTC_DFER |= 1 << 7;
  // BTN1
  PORTE_PCR26 = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_MUX(1);
  PORTE_DFER |= 1 << 26;
  // BTN2
  PORTA_PCR14 = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_MUX(1);
  PORTA_DFER |= 1 << 14;
  // BTN3
  PORTD_PCR0 = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_MUX(1);
  PORTD_DFER |= 1 << 0;
  // BTN4
  PORTD_PCR7 = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_MUX(1);
  PORTD_DFER |= 1 << 7;
  // BTN5 (only new revision)
  PORTA_PCR15 = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_MUX(1);
  PORTA_DFER |= 1 << 15;

  PORTA_PCR5 = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_MUX(1);

  DMA_CR = DMA_CR_EMLM;

  teensy::UsbDevice usb_device(0, 0x16c0, 0x0490);
  usb_device.SetManufacturer("FRC 971 Spartan Robotics");
  usb_device.SetProduct("Pistol Grip Controller debug");
  teensy::AcmTty tty1(&usb_device);
  teensy::AcmTty tty2(&usb_device);
  global_stdout.store(&tty1, ::std::memory_order_release);
  usb_device.Initialize();

  AdcInitSmall();
  MathInit();
  delay(100);
  can_init(2, 3);

  GPIOD_PCOR = 1 << 3;
  PERIPHERAL_BITBAND(GPIOD_PDDR, 3) = 1;
  PORTD_PCR3 = PORT_PCR_DSE | PORT_PCR_MUX(1);
  GPIOD_PSOR = 1 << 3;

  GPIOC_PCOR = 1 << 4;
  PERIPHERAL_BITBAND(GPIOC_PDDR, 4) = 1;
  PORTC_PCR4 = PORT_PCR_DSE | PORT_PCR_MUX(1);
  GPIOC_PSOR = 1 << 4;

  LittleMotorControlsImplementation controls0, controls1;

  delay(100);

  // M0_EA = FTM1_QD_PHB
  PORTB_PCR19 = PORT_PCR_MUX(6);
  // M0_EB = FTM1_QD_PHA
  PORTB_PCR18 = PORT_PCR_MUX(6);

  // M1_EA = FTM1_QD_PHA
  PORTB_PCR0 = PORT_PCR_MUX(6);
  // M1_EB = FTM1_QD_PHB
  PORTB_PCR1 = PORT_PCR_MUX(6);

  // M0_CH0 = FTM3_CH4
  PORTC_PCR8 = PORT_PCR_DSE | PORT_PCR_MUX(3);
  // M0_CH1 = FTM3_CH2
  PORTD_PCR2 = PORT_PCR_DSE | PORT_PCR_MUX(4);
  // M0_CH2 = FTM3_CH6
  PORTC_PCR10 = PORT_PCR_DSE | PORT_PCR_MUX(3);

  // M1_CH0 = FTM0_CH0
  PORTC_PCR1 = PORT_PCR_DSE | PORT_PCR_MUX(4);
  // M1_CH1 = FTM0_CH2
  PORTC_PCR3 = PORT_PCR_DSE | PORT_PCR_MUX(4);
  // M1_CH2 = FTM0_CH4
  PORTD_PCR4 = PORT_PCR_DSE | PORT_PCR_MUX(4);

  Motor motor0(
      MOTOR0_PWM_FTM, MOTOR0_ENCODER_FTM, &controls0,
      {&MOTOR0_PWM_FTM->C4V, &MOTOR0_PWM_FTM->C2V, &MOTOR0_PWM_FTM->C6V});
  motor0.set_debug_tty(&tty2);
  motor0.set_switching_divisor(kSwitchingDivisor);
  Motor motor1(
      MOTOR1_PWM_FTM, MOTOR1_ENCODER_FTM, &controls1,
      {&MOTOR1_PWM_FTM->C0V, &MOTOR1_PWM_FTM->C2V, &MOTOR1_PWM_FTM->C4V});
  motor1.set_debug_tty(&tty2);
  motor1.set_switching_divisor(kSwitchingDivisor);
  ConfigurePwmFtm(MOTOR0_PWM_FTM);
  ConfigurePwmFtm(MOTOR1_PWM_FTM);
  motor0.Init();
  motor1.Init();
  global_motor0.store(&motor0, ::std::memory_order_relaxed);
  global_motor1.store(&motor1, ::std::memory_order_relaxed);

  SIM_SCGC6 |= SIM_SCGC6_PIT;
  // Workaround for errata e7914.
  (void)PIT_MCR;
  PIT_MCR = 0;
  PIT_LDVAL3 = (BUS_CLOCK_FREQUENCY / 1000) - 1;
  PIT_TCTRL3 = PIT_TCTRL_TIE | PIT_TCTRL_TEN;

  // Have them both wait for the GTB signal.
  FTM0->CONF = FTM3->CONF =
      FTM_CONF_GTBEEN | FTM_CONF_NUMTOF(kSwitchingDivisor - 1);
  // Make FTM3's period half of what it should be so we can get it a half-cycle
  // out of phase.
  const uint32_t original_mod = FTM3->MOD;
  FTM3->MOD = ((original_mod + 1) / 2) - 1;
  FTM3->SYNC |= FTM_SYNC_SWSYNC;

  // Output triggers to things like the PDBs on initialization.
  FTM0_EXTTRIG = FTM_EXTTRIG_INITTRIGEN;
  FTM3_EXTTRIG = FTM_EXTTRIG_INITTRIGEN;
  // Don't let any memory accesses sneak past here, because we actually
  // need everything to be starting up.
  __asm__("" ::: "memory");

  // Give everything a chance to get going.
  delay(100);

  printf("BSS: %p-%p\n", __bss_ram_start__, __bss_ram_end__);
  printf("data: %p-%p\n", __data_ram_start__, __data_ram_end__);
  printf("heap start: %p\n", __heap_start__);
  printf("stack start: %p\n", __stack_end__);

  printf("Zeroing motors\n");
  uint16_t motor0_offset, motor1_offset, wheel_offset;
  while (!ZeroMotors(&motor0_offset, &motor1_offset, &wheel_offset)) {
  }
  printf("Done zeroing\n");

  const float motor0_offset_scaled = -analog_ratio(motor0_offset);
  const float motor1_offset_scaled = analog_ratio(motor1_offset);
  // Good for the initial trigger.
  {
    constexpr float kZeroOffset0 = 0.27f;
    const int motor0_starting_point = static_cast<int>(
        (motor0_offset_scaled + (kZeroOffset0 / 7.0f)) * 4096.0f);
    printf("Motor 0 starting at %d\n", motor0_starting_point);
    motor0.set_encoder_calibration_offset(motor0_starting_point);
    motor0.set_encoder_multiplier(-1);

    // Calibrate neutral here.
    motor0.set_encoder_offset(motor0.encoder_offset() - 2065 + 20);

    uint32_t new_encoder = motor0.wrapped_encoder();
    int32_t absolute_encoder = motor0.absolute_encoder(new_encoder);
    printf("Motor 0 encoder %d absolute %d\n", static_cast<int>(new_encoder),
           static_cast<int>(absolute_encoder));
  }

  {
    constexpr float kZeroOffset1 = 0.26f;
    const int motor1_starting_point = static_cast<int>(
        (motor1_offset_scaled + (kZeroOffset1 / 7.0f)) * 4096.0f);
    printf("Motor 1 starting at %d\n", motor1_starting_point);
    motor1.set_encoder_calibration_offset(motor1_starting_point);
    motor1.set_encoder_multiplier(-1);

    float wheel_position = absolute_wheel(analog_ratio(wheel_offset));

    uint32_t encoder = motor1.wrapped_encoder();

    printf("Wheel starting at %d, encoder %" PRId32 "\n",
           static_cast<int>(wheel_position * 1000.0f), encoder);

    constexpr float kWheelGearRatio = (1.25f + 0.02f) / 0.35f;
    constexpr float kWrappedWheelAtZero = 0.6586310546875f;

    const int encoder_wraps =
        static_cast<int>(lround(wheel_position * kWheelGearRatio -
                                (encoder / 4096.f) + kWrappedWheelAtZero));

    printf("Wraps: %d\n", encoder_wraps);
    motor1.set_encoder_offset(4096 * encoder_wraps + motor1.encoder_offset() -
                              static_cast<int>(kWrappedWheelAtZero * 4096));
    printf("Wheel encoder now at %d\n",
           static_cast<int>(1000.f / 4096.f *
                            motor1.absolute_encoder(motor1.wrapped_encoder())));
  }

  // Turn an LED on for Austin.
  PERIPHERAL_BITBAND(GPIOC_PDDR, 6) = 1;
  GPIOC_PCOR = 1 << 6;
  PORTC_PCR6 = PORT_PCR_DSE | PORT_PCR_MUX(1);

  // M0_THW
  PORTC_PCR11 = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_MUX(1);
  // M0_FAULT
  PORTD_PCR6 = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_MUX(1);
  // M1_THW
  PORTC_PCR2 = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_MUX(1);
  // M1_FAULT
  PORTD_PCR5 = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_MUX(1);

  motor0.Start();
  motor1.Start();
  {
    // We rely on various things happening faster than the timer period, so make
    // sure slow USB or whatever interrupts don't prevent that.
    DisableInterrupts disable_interrupts;

    // First clear the overflow flag.
    FTM3->SC &= ~FTM_SC_TOF;

    // Now poke the GTB to actually start both timers.
    FTM0->CONF = FTM_CONF_GTBEEN | FTM_CONF_GTBEOUT |
                 FTM_CONF_NUMTOF(kSwitchingDivisor - 1);

    // Wait for it to overflow twice. For some reason, just once doesn't work.
    while (!(FTM3->SC & FTM_SC_TOF)) {
    }
    FTM3->SC &= ~FTM_SC_TOF;
    while (!(FTM3->SC & FTM_SC_TOF)) {
    }

    // Now put the MOD value back to what it was.
    FTM3->MOD = original_mod;
    FTM3->PWMLOAD = FTM_PWMLOAD_LDOK;

    // And then clear the overflow flags before enabling interrupts so we
    // actually wait until the next overflow to start doing interrupts.
    FTM0->SC &= ~FTM_SC_TOF;
    FTM3->SC &= ~FTM_SC_TOF;
    NVIC_ENABLE_IRQ(IRQ_FTM0);
    NVIC_ENABLE_IRQ(IRQ_FTM3);
  }
  global_trigger_plant.store(
      new StateFeedbackPlant<3, 1, 1, float>(MakeIntegralHapticTriggerPlant()));
  global_trigger_observer.store(new StateFeedbackObserver<3, 1, 1, float>(
      MakeIntegralHapticTriggerObserver()));
  global_trigger_observer.load(::std::memory_order_relaxed)
      ->Reset(global_trigger_plant.load(::std::memory_order_relaxed));

  global_wheel_plant.store(
      new StateFeedbackPlant<3, 1, 1, float>(MakeIntegralHapticWheelPlant()));
  global_wheel_observer.store(new StateFeedbackObserver<3, 1, 1, float>(
      MakeIntegralHapticWheelObserver()));
  global_wheel_observer.load(::std::memory_order_relaxed)
      ->Reset(global_wheel_plant.load(::std::memory_order_relaxed));

  delay(1000);

  NVIC_ENABLE_IRQ(IRQ_PIT_CH3);

  // TODO(Brian): Use SLEEPONEXIT to reduce interrupt latency?
  while (true) {
    if (!PERIPHERAL_BITBAND(GPIOC_PDIR, 11)) {
      if (!PERIPHERAL_BITBAND(GPIOC_PDOR, 5)) {
        printf("M0_THW\n");
      }
      GPIOC_PSOR = 1 << 5;
    }
    if (!PERIPHERAL_BITBAND(GPIOD_PDIR, 6)) {
      if (!PERIPHERAL_BITBAND(GPIOC_PDOR, 5)) {
        printf("M0_FAULT\n");
      }
      GPIOC_PSOR = 1 << 5;
    }
    if (!PERIPHERAL_BITBAND(GPIOC_PDIR, 2)) {
      if (!PERIPHERAL_BITBAND(GPIOC_PDOR, 5)) {
        printf("M1_THW\n");
      }
      GPIOC_PSOR = 1 << 5;
    }
    if (!PERIPHERAL_BITBAND(GPIOD_PDIR, 5)) {
      if (!PERIPHERAL_BITBAND(GPIOC_PDOR, 5)) {
        printf("M1_FAULT\n");
      }
      GPIOC_PSOR = 1 << 5;
    }
  }

  return 0;
}

}  // namespace motors
}  // namespace frc971
