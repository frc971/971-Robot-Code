#include "motors/core/kinetis.h"

#include <inttypes.h>
#include <stdio.h>

#include <atomic>
#include <cmath>

#include "frc971/control_loops/drivetrain/integral_haptic_trigger.h"
#include "frc971/control_loops/drivetrain/integral_haptic_wheel.h"
#include "motors/core/time.h"
#include "motors/motor.h"
#include "motors/peripheral/can.h"
#include "motors/pistol_grip/controller_adc.h"
#include "motors/pistol_grip/motor_controls.h"
#include "motors/print/print.h"
#include "motors/util.h"

#define MOTOR0_PWM_FTM FTM3
#define MOTOR0_ENCODER_FTM FTM2
#define MOTOR1_PWM_FTM FTM0
#define MOTOR1_ENCODER_FTM FTM1

extern const float kWheelCoggingTorque0[4096];
extern const float kWheelCoggingTorque1[4096];
extern const float kTriggerCoggingTorque0[4096];
extern const float kTriggerCoggingTorque1[4096];

namespace frc971 {
namespace motors {
namespace {

::std::atomic<const float *> trigger_cogging_torque{nullptr};
::std::atomic<const float *> wheel_cogging_torque{nullptr};

float TriggerCoggingTorque(uint32_t index) {
  return trigger_cogging_torque.load(::std::memory_order_relaxed)[index];
}

float WheelCoggingTorque(uint32_t index) {
  return wheel_cogging_torque.load(::std::memory_order_relaxed)[index];
}

using ::frc971::control_loops::drivetrain::MakeIntegralHapticTriggerPlant;
using ::frc971::control_loops::drivetrain::MakeIntegralHapticTriggerObserver;
using ::frc971::control_loops::drivetrain::MakeIntegralHapticWheelPlant;
using ::frc971::control_loops::drivetrain::MakeIntegralHapticWheelObserver;

// Returns an identifier for the processor we're running on.
// This isn't guaranteed to be unique, but it should be close enough.
uint8_t ProcessorIdentifier() {
  // This XORs together all the bytes of the unique identifier provided by the
  // hardware.
  uint8_t r = 0;
  for (uint8_t uid : {SIM_UIDH, SIM_UIDMH, SIM_UIDML, SIM_UIDL}) {
    r = r ^ ((uid >> 0) & 0xFF);
    r = r ^ ((uid >> 8) & 0xFF);
    r = r ^ ((uid >> 16) & 0xFF);
    r = r ^ ((uid >> 24) & 0xFF);
  }
  return r;
}

uint8_t ProcessorIndex() {
  switch (ProcessorIdentifier()) {
    case static_cast<uint8_t>(0xaa):
      return 1;
    default:
      return 0;
  }
}

// Cached version for speed.
const uint8_t processor_index = ProcessorIndex();

constexpr float kHapticWheelCurrentLimit = static_cast<float>(
    ::frc971::control_loops::drivetrain::kHapticWheelCurrentLimit);
constexpr float kHapticTriggerCurrentLimit = static_cast<float>(
    ::frc971::control_loops::drivetrain::kHapticTriggerCurrentLimit);

::std::atomic<Motor *> global_motor0{nullptr}, global_motor1{nullptr};

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
::std::atomic<float> global_trigger_current{0.0f};

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
  const float kCenterOffset = (processor_index == 1) ? -0.683f : -0.935f;

  wheel_position += kCenterOffset;

  if (wheel_position > 0.5f) {
    wheel_position -= 1.0f;
  } else if (wheel_position < -0.5f) {
    wheel_position += 1.0f;
  }
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

float CoggingCurrent1(uint32_t encoder, int32_t absolute_encoder) {
  constexpr float kP = 0.05f;
  constexpr float kI = 0.00001f;
  static int goal = -6700;

  const int error = goal - static_cast<int>(absolute_encoder);
  static float error_sum = 0.0f;
  float goal_current = static_cast<float>(error) * kP + error_sum * kI;

  goal_current = ::std::min(1.0f, ::std::max(-1.0f, goal_current));

  static int i = 0;
  if (error == 0) {
    ++i;
  } else {
    i = 0;
  }
  if (i >= 100) {
    printf("reading1: %d %d a:%d e:%d\n", goal,
           static_cast<int>(goal_current * 10000.0f),
           static_cast<int>(encoder),
           static_cast<int>(error));
    static int counting_up = 0;
    if (absolute_encoder <= -6900) {
      counting_up = 1;
    } else if (absolute_encoder >= 6900) {
      counting_up = 0;
    }
    if (counting_up) {
      ++goal;
    } else {
      --goal;
    }
    i = 0;
  }

  error_sum += static_cast<float>(error);
  if (error_sum > 1.0f / kI) {
    error_sum = 1.0f / kI;
  } else if (error_sum < -1.0f / kI) {
    error_sum = -1.0f / kI;
  }
  return goal_current;
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

  (void)CoggingCurrent1;
  float goal_current = global_wheel_current.load(::std::memory_order_relaxed) +
                       WheelCoggingTorque(encoder);
  //float goal_current = CoggingCurrent1(encoder, absolute_encoder);
  // float goal_current = kWheelCoggingTorque[encoder];
  // float goal_current = 0.0f;

  // Controller 0 is mechanical and doesn't need the motor controls.
  if (processor_index == 0) {
    global_motor1.load(::std::memory_order_relaxed)->CycleFixedPhaseInterupt(0);
  } else {
    global_motor1.load(::std::memory_order_relaxed)
        ->SetGoalCurrent(goal_current);
    global_motor1.load(::std::memory_order_relaxed)
        ->CurrentInterrupt(BalanceSimpleReadings(readings.currents), encoder);
    global_wheel_angle.store(angle);
  }
  //global_motor1.load(::std::memory_order_relaxed)->CycleFixedPhaseInterupt();


  /*
  SmallInitReadings position_readings;
  {
    DisableInterrupts disable_interrupts;
    position_readings = AdcReadSmallInit(disable_interrupts);
  }

  static int i = 0;
  if (i == 1000) {
    i = 0;
    float wheel_position =
        absolute_wheel(analog_ratio(position_readings.wheel_abs));
    printf(
        "ecnt %" PRIu32 " arev:%d erev:%d abs:%d awp:%d uncalwheel:%d\n",
        encoder,
        static_cast<int>((1.0f - analog_ratio(position_readings.motor1_abs)) *
                         7000.0f),
        static_cast<int>(encoder * 7.0f / 4096.0f * 1000.0f),
        static_cast<int>(absolute_encoder),
        static_cast<int>(wheel_position * 1000.0f),
        static_cast<int>(analog_ratio(position_readings.wheel_abs) * 1000.0f));
  } else if (i == 200) {
    printf("out %" PRIu32 " %" PRIu32 " %" PRIu32 "\n",
           global_motor1.load(::std::memory_order_relaxed)
               ->output_registers()[0][2],
           global_motor1.load(::std::memory_order_relaxed)
               ->output_registers()[1][2],
           global_motor1.load(::std::memory_order_relaxed)
               ->output_registers()[2][2]);
  }
  ++i;
  */
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

float CoggingCurrent0(uint32_t encoder, int32_t absolute_encoder) {
  constexpr float kP = 0.05f;
  constexpr float kI = 0.00001f;
  static int goal = 0;

  const int error = goal - static_cast<int>(absolute_encoder);
  static float error_sum = 0.0f;
  float goal_current = static_cast<float>(error) * kP + error_sum * kI;

  goal_current = ::std::min(1.0f, ::std::max(-1.0f, goal_current));

  static int i = 0;
  if (error == 0) {
    ++i;
  } else {
    i = 0;
  }

  if (i >= 100) {
    printf("reading0: %d %d a:%d e:%d\n", goal,
           static_cast<int>(goal_current * 10000.0f),
           static_cast<int>(encoder),
           static_cast<int>(error));
    static int counting_up = 0;
    if (absolute_encoder <= -1390) {
      counting_up = 1;
    } else if (absolute_encoder >= 1390) {
      counting_up = 0;
    }
    if (counting_up) {
      ++goal;
    } else {
      --goal;
    }
  }

  error_sum += static_cast<float>(error);
  if (error_sum > 1.0f / kI) {
    error_sum = 1.0f / kI;
  } else if (error_sum < -1.0f / kI) {
    error_sum = -1.0f / kI;
  }
  return goal_current;
}

extern "C" void ftm3_isr() {
  SmallAdcReadings readings;
  {
    DisableInterrupts disable_interrupts;
    readings = AdcReadSmall0(disable_interrupts);
  }

  const uint32_t encoder =
      global_motor0.load(::std::memory_order_relaxed)->wrapped_encoder();
  const int32_t absolute_encoder =
      global_motor0.load(::std::memory_order_relaxed)
          ->absolute_encoder(encoder);

  const float trigger_angle = absolute_encoder / 1370.f;

  (void)CoggingCurrent0;
  const float goal_current =
      global_trigger_current.load(::std::memory_order_relaxed) +
      TriggerCoggingTorque(encoder);
  //const float goal_current = kTriggerCoggingTorque[encoder];
  //const float goal_current = 0.0f;
  //const float goal_current = CoggingCurrent0(encoder, absolute_encoder);

  if (processor_index == 0) {
    global_motor0.load(::std::memory_order_relaxed)->CycleFixedPhaseInterupt(0);
  } else {
    global_motor0.load(::std::memory_order_relaxed)
        ->SetGoalCurrent(goal_current);
    global_motor0.load(::std::memory_order_relaxed)
        ->CurrentInterrupt(BalanceSimpleReadings(readings.currents), encoder);
    global_trigger_angle.store(trigger_angle);
  }
  //global_motor0.load(::std::memory_order_relaxed)->CycleFixedPhaseInterupt();


  /*
  SmallInitReadings position_readings;
  {
    DisableInterrupts disable_interrupts;
    position_readings = AdcReadSmallInit(disable_interrupts);
  }

  static int i = 0;
  if (i == 1000) {
    i = 0;
    printf("ecnt %" PRIu32 " arev:%d erev:%d abs:%d\n", encoder,
           static_cast<int>((analog_ratio(position_readings.motor0_abs)) *
                            7000.0f),
           static_cast<int>(encoder * 7.0f / 4096.0f * 1000.0f),
           static_cast<int>(absolute_encoder));
  } else if (i == 200) {
    printf("out %" PRIu32 " %" PRIu32 " %" PRIu32 "\n",
           global_motor0.load(::std::memory_order_relaxed)
               ->output_registers()[0][2],
           global_motor0.load(::std::memory_order_relaxed)
               ->output_registers()[1][2],
           global_motor0.load(::std::memory_order_relaxed)
               ->output_registers()[2][2]);
  }
  ++i;
  */
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

  // If we are a mechanical pistol grip, do the encoder angle calculation here
  // since the high frequency interrupt isn't running.
  if (processor_index == 0) {
    const uint32_t trigger_encoder =
        global_motor0.load(::std::memory_order_relaxed)->wrapped_encoder();
    const int32_t trigger_absolute_encoder =
        global_motor0.load(::std::memory_order_relaxed)
            ->absolute_encoder(trigger_encoder);

    const float trigger_angle =
        trigger_absolute_encoder /
        (trigger_absolute_encoder < 0.0 ? 198.0f : 252.0f);
    global_trigger_angle.store(
        ::std::max(-1.0f, ::std::min(1.0f, trigger_angle)));

    uint32_t wheel_encoder =
        global_motor1.load(::std::memory_order_relaxed)->wrapped_encoder();
    int32_t wheel_absolute_encoder =
        global_motor1.load(::std::memory_order_relaxed)
            ->absolute_encoder(wheel_encoder);

    const float wheel_angle = wheel_absolute_encoder / 1787.0f;
    global_wheel_angle.store(::std::max(-1.0f, ::std::min(1.0f, wheel_angle)));

    /*
    // Calibration constants
    static int k = 0;
    ++k;
    if (k > 100) {
      printf("trigger: %d -> %d  wheel %d -> %d -> %d\n",
             static_cast<int>(trigger_encoder),
             static_cast<int>(global_trigger_angle * 1000.0f),
             static_cast<int>(wheel_encoder),
             static_cast<int>(wheel_absolute_encoder),
             static_cast<int>(global_wheel_angle * 1000.0f));
      k = 0;
    }
    */
  }

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
          static_cast<float>(32768.0 * M_PI / 8.0);
      trigger_goal_velocity =
          static_cast<float>(
              static_cast<int32_t>(static_cast<uint32_t>(data[2]) |
                                   (static_cast<uint32_t>(data[3]) << 8)) -
              32768) /
          static_cast<float>(32768.0 * 4.0);

      trigger_haptic_current =
          static_cast<float>(
              static_cast<int32_t>(static_cast<uint32_t>(data[4]) |
                                   (static_cast<uint32_t>(data[5]) << 8)) -
              32768) /
          static_cast<float>(32768.0 * 2.0);
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
          static_cast<float>(32768.0 * M_PI);
      wheel_goal_velocity =
          static_cast<float>(
              static_cast<int32_t>(static_cast<uint32_t>(data[2]) |
                                   (static_cast<uint32_t>(data[3]) << 8)) -
              32768) /
          static_cast<float>(32768.0 * 10.0);

      wheel_haptic_current =
          static_cast<float>(
              static_cast<int32_t>(static_cast<uint32_t>(data[4]) |
                                   (static_cast<uint32_t>(data[5]) << 8)) -
              32768) /
          static_cast<float>(32768.0 * 2.0);
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

  if (processor_index == 0) {
    wheel_goal_current = 0.0f;
    trigger_goal_current = 0.0f;
  }
  global_wheel_current.store(wheel_goal_current, ::std::memory_order_relaxed);
  global_trigger_current.store(trigger_goal_current,
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

  DMA.CR = M_DMA_EMLM;

  PrintingParameters printing_parameters;
  printing_parameters.dedicated_usb = true;
  const ::std::unique_ptr<PrintingImplementation> printing =
      CreatePrinting(printing_parameters);
  printing->Initialize();

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
  motor0.set_printing_implementation(printing.get());
  motor0.set_switching_divisor(kSwitchingDivisor);
  Motor motor1(
      MOTOR1_PWM_FTM, MOTOR1_ENCODER_FTM, &controls1,
      {&MOTOR1_PWM_FTM->C0V, &MOTOR1_PWM_FTM->C2V, &MOTOR1_PWM_FTM->C4V});
  motor1.set_printing_implementation(printing.get());
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

  trigger_cogging_torque.store(processor_index == 0 ? kTriggerCoggingTorque0
                                                     : kTriggerCoggingTorque1);
  wheel_cogging_torque.store(processor_index == 0 ? kWheelCoggingTorque0
                                                   : kWheelCoggingTorque1);

  printf("Zeroing motors for %d:%x\n", static_cast<int>(processor_index),
         (unsigned int)ProcessorIdentifier());
  uint16_t motor0_offset, motor1_offset, wheel_offset;
  while (!ZeroMotors(&motor0_offset, &motor1_offset, &wheel_offset)) {
  }
  printf("Done zeroing\n");

  const float motor0_offset_scaled = -analog_ratio(motor0_offset);
  const float motor1_offset_scaled = analog_ratio(motor1_offset);
  // Good for the initial trigger.
  {
    // Calibrate winding phase error here.
    const float kZeroOffset0 = processor_index == 1 ? 0.275f : 0.0f;
    const int motor0_starting_point = static_cast<int>(
        (motor0_offset_scaled + (kZeroOffset0 / 7.0f)) * 4096.0f);
    printf("Motor 0 starting at %d\n", motor0_starting_point);
    motor0.set_encoder_calibration_offset(motor0_starting_point);
    motor0.set_encoder_multiplier(-1);

    // Calibrate output coordinate neutral here.
    motor0.set_encoder_offset(
        motor0.encoder_offset() +
        (processor_index == 1 ? (-3096 - 430 - 30 - 6) : (-1978)));

    while (true) {
      const uint32_t encoder =
          global_motor0.load(::std::memory_order_relaxed)->wrapped_encoder();
      const int32_t absolute_encoder =
          global_motor0.load(::std::memory_order_relaxed)
              ->absolute_encoder(encoder);

      if (absolute_encoder > 2047) {
        motor0.set_encoder_offset(motor0.encoder_offset() - 4096);
      } else if (absolute_encoder < -2047) {
        motor0.set_encoder_offset(motor0.encoder_offset() + 4096);
      } else {
        break;
      }
    }

    uint32_t new_encoder = motor0.wrapped_encoder();
    int32_t absolute_encoder = motor0.absolute_encoder(new_encoder);
    printf("Motor 0 encoder %d absolute %d\n", static_cast<int>(new_encoder),
           static_cast<int>(absolute_encoder));
  }

  {
    const float kZeroOffset1 = processor_index == 1 ? 0.420f : 0.0f;

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
    const float kWrappedWheelAtZero =
        processor_index == 1 ? (0.934630859375f) : (1809.f / 4096.f);

    // If we are pistol grip 0, we are the mechanical pistol grip and can't
    // wrap.  People are very very likely to zero it at the zero position too.
    const int encoder_wraps =
        processor_index == 0 ? 0
                             : static_cast<int>(lround(
                                   wheel_position * kWheelGearRatio -
                                   (encoder / 4096.f) + kWrappedWheelAtZero));

    printf("Wraps: %d\n", encoder_wraps);
    motor1.set_encoder_offset(4096 * encoder_wraps + motor1.encoder_offset() -
                              static_cast<int>(kWrappedWheelAtZero * 4096.0f));
    printf("Wheel encoder now at %d\n",
           static_cast<int>(motor1.absolute_encoder(motor1.wrapped_encoder())));
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
