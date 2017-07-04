#ifndef MOTORS_MOTOR_H_
#define MOTORS_MOTOR_H_

#include <limits.h>

#include <array>

#include "motors/algorithms.h"
#include "motors/core/kinetis.h"
#include "motors/peripheral/adc.h"
#include "motors/util.h"

namespace frc971 {
namespace salsa {

class MotorControls {
 public:
  MotorControls() = default;
  virtual ~MotorControls() = default;

  MotorControls(const MotorControls &) = delete;
  void operator=(const MotorControls &) = delete;

  static constexpr float scale_current_reading(float reading) {
    return reading *
           static_cast<float>(1.0 / 4096.0 /* Full-scale ADC reading */ *
                              3.3 /* ADC reference voltage */ /
                              (1.47 / (0.768 + 1.47)) /* 5V -> 3.3V divider */ /
                              50.0 /* Current sense amplification */ /
                              0.0003 /* Sense resistor */);
  }

  static constexpr int counts_per_revolution() { return 2048 / 2; }

  // raw_currents are in amps for each phase.
  // theta is in electrical counts, which will be less than
  // counts_per_revolution().
  virtual ::std::array<uint32_t, 3> DoIteration(
      const float raw_currents[3], uint32_t theta,
      const float command_current) = 0;

  virtual int16_t Debug(uint32_t theta) = 0;

  virtual float estimated_velocity() const = 0;
};

// Controls a single motor.
class Motor {
 public:
  // pwm_ftm is used to drive the PWM outputs.
  // encoder_ftm is used for reading the encoder.
  Motor(BigFTM *pwm_ftm, LittleFTM *encoder_ftm, MotorControls *controls);

  Motor(const Motor &) = delete;
  void operator=(const Motor &) = delete;

  // Sets up everything but doesn't actually start the timers.
  //
  // This assumes the global time base configuration happens outside so the
  // timers for both motors (if applicable) are synced up.
  // TODO(Brian): "40.4.28.1 Enabling the global time base (GTB)" says something
  // about needing to do stuff in a specific order, so this API might need
  // revising.
  void Init();

  // Zeros the encoder. This involves blocking for an arbitrary length of time
  // with interrupts disabled.
  void Zero();

  // Starts the timers.
  void Start();

  void HandleInterrupt();

 private:
  // Represents the ADC reading which is closest to an edge.
  struct CloseAdcReading {
    // Adds a new reading to the readings to balance or pushes the previous
    // closest one there and saves off this one.
    //
    // Returns true if it saves off the new reading.
    bool MaybeUse(int new_distance, const MediumAdcReadings &adc_readings,
                  int phase, int sample, ReadingsToBalance *to_balance) {
      if (new_distance < distance) {
        if (distance != INT_MAX) {
          to_balance->Add(index, value);
        }
        distance = new_distance;
        value = adc_readings.motor_currents[phase][sample];
        index = phase;
        return true;
      }
      return false;
    }

    int distance = INT_MAX;
    int32_t value = 0;
    int index = 0;
  };

  bool flip_time_offset_ = false;

  BigFTM *const pwm_ftm_;
  LittleFTM *const encoder_ftm_;
  MotorControls *const controls_;
};

}  // namespace salsa
}  // namespace frc971

#endif  // MOTORS_MOTOR_H_
