#ifndef MOTORS_MOTOR_H_
#define MOTORS_MOTOR_H_

#include <limits.h>

#include <array>

#include "motors/algorithms.h"
#include "motors/core/kinetis.h"
#include "motors/core/time.h"
#include "motors/peripheral/adc.h"
#include "motors/peripheral/configuration.h"
#include "motors/print/print.h"
#include "motors/util.h"

namespace frc971 {
namespace motors {

class MotorControls {
 public:
  MotorControls() = default;
  virtual ~MotorControls() = default;

  MotorControls(const MotorControls &) = delete;
  void operator=(const MotorControls &) = delete;

  virtual void Reset() = 0;

  // Scales a current reading from ADC units to amps.
  //
  // Note that this doesn't apply any offset. The common offset will be
  // automatically removed as part of the balancing process.
  virtual float scale_current_reading(float reading) const = 0;

  virtual int mechanical_counts_per_revolution() const = 0;
  virtual int electrical_counts_per_revolution() const = 0;

  // raw_currents are in amps for each phase.
  // theta is in electrical counts, which will be less than
  // counts_per_revolution().
  virtual ::std::array<float, 3> DoIteration(const float raw_currents[3],
                                             uint32_t theta,
                                             const float command_current) = 0;

  virtual int16_t Debug(uint32_t theta) = 0;

  virtual float estimated_velocity() const = 0;
  virtual int16_t i_goal(size_t ii) const = 0;
  virtual float overall_measured_current() const = 0;
};

// Controls a single motor.
class Motor final {
 public:
  // pwm_ftm is used to drive the PWM outputs.
  // encoder_ftm is used for reading the encoder.
  Motor(BigFTM *pwm_ftm, LittleFTM *encoder_ftm, MotorControls *controls,
        const ::std::array<volatile uint32_t *, 3> &output_registers);

  Motor(const Motor &) = delete;
  void operator=(const Motor &) = delete;

  void Reset() { controls_->Reset(); }

  void set_printing_implementation(
      PrintingImplementation *printing_implementation) {
    printing_implementation_ = printing_implementation;
  }
  void set_deadtime_compensation(int deadtime_compensation) {
    deadtime_compensation_ = deadtime_compensation;
  }
  void set_switching_divisor(int switching_divisor) {
    switching_divisor_ = switching_divisor;
  }
  void set_encoder_offset(int32_t encoder_offset) {
    encoder_offset_ = encoder_offset;
    last_wrapped_encoder_reading_ = wrapped_encoder();
  }
  int32_t encoder_offset() const { return encoder_offset_; }

  void set_encoder_calibration_offset(int encoder_offset) {
    encoder_calibration_offset_ = encoder_offset;
    // Add mechanical_counts_per_revolution to the offset so that when we mod
    // below, we are guaranteed to be > 0 regardless of the encoder multiplier.
    // % isn't well-defined with negative numbers.
    while (encoder_calibration_offset_ <
           controls_->mechanical_counts_per_revolution()) {
      encoder_calibration_offset_ +=
          controls_->mechanical_counts_per_revolution();
    }
  }
  void set_encoder_multiplier(int encoder_multiplier) {
    encoder_multiplier_ = encoder_multiplier;
  }

  int32_t absolute_encoder(uint32_t wrapped_encoder_reading) {
    const uint32_t counts_per_revolution =
        controls_->mechanical_counts_per_revolution();
    const uint32_t wrap_down = counts_per_revolution / 4;
    const uint32_t wrap_up = wrap_down * 3;
    if (last_wrapped_encoder_reading_ > wrap_up &&
        wrapped_encoder_reading < wrap_down) {
      encoder_offset_ += counts_per_revolution;
    } else if (last_wrapped_encoder_reading_ < wrap_down &&
               wrapped_encoder_reading > wrap_up) {
      encoder_offset_ -= counts_per_revolution;
    }

    last_wrapped_encoder_reading_ = wrapped_encoder_reading;

    return static_cast<int32_t>(wrapped_encoder_reading) + encoder_offset_;
  }

  int encoder() {
    return encoder_multiplier_ * encoder_ftm_->CNT;
  }
  uint32_t wrapped_encoder() {
    return (encoder() + encoder_calibration_offset_) %
           controls_->mechanical_counts_per_revolution();
  }

  // Sets up everything but doesn't actually start the timers.
  //
  // This assumes the global time base configuration happens outside so the
  // timers for both motors (if applicable) are synced up.
  void Init();

  // Starts the timers.
  //
  // If the global time base is in use, it must be activated after this.
  void Start();

  void CurrentInterrupt(const BalancedReadings &readings,
                        uint32_t captured_wrapped_encoder);

  // Runs each phase at a fixed duty cycle.
  void CycleFixedPhaseInterupt(int period = 80);

  void SetGoalCurrent(float goal_current) {
    DisableInterrupts disable_interrupts;
    goal_current_ = goal_current;
    last_current_set_time_ = micros();
  }

  inline int counts_per_cycle() const {
    return BUS_CLOCK_FREQUENCY / SWITCHING_FREQUENCY / switching_divisor_;
  }

  inline uint16_t get_switching_points_cycles(size_t ii) const {
    return static_cast<uint16_t>(switching_points_ratio_[ii] *
                                 counts_per_cycle());
  }

  inline float estimated_velocity() const {
    return controls_->estimated_velocity();
  }

  inline int16_t i_goal(size_t ii) const {
    return controls_->i_goal(ii);
  }

  inline int16_t goal_current() const {
    return goal_current_;
  }

  inline float overall_measured_current() const {
    return controls_->overall_measured_current();
  }

  ::std::array<volatile uint32_t *, 3> output_registers() const {
    return output_registers_;
  }

 private:
  uint32_t CalculateOnTime(uint32_t width) const;
  uint32_t CalculateOffTime(uint32_t width) const;

  bool flip_time_offset_ = false;
  int deadtime_compensation_ = 0;

  BigFTM *const pwm_ftm_;
  LittleFTM *const encoder_ftm_;
  MotorControls *const controls_;
  const ::std::array<volatile uint32_t *, 3> output_registers_;
  ::std::array<float, 3> switching_points_ratio_;

  float goal_current_ = 0;
  uint32_t last_current_set_time_ = 0;
  int switching_divisor_ = 1;
  int encoder_calibration_offset_ = 0;
  int32_t encoder_offset_ = 0;
  int encoder_multiplier_ = 1;
  uint32_t last_wrapped_encoder_reading_ = 0;

  PrintingImplementation *printing_implementation_ = nullptr;
};

}  // namespace motors
}  // namespace frc971

#endif  // MOTORS_MOTOR_H_
