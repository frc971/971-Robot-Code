#include "motors/motor.h"

#include <limits.h>
#include <stdio.h>
#include <inttypes.h>

#include <array>

#include "motors/peripheral/configuration.h"
#include "motors/peripheral/can.h"

extern "C" float analog_ratio(uint16_t reading);
extern "C" float absolute_wheel(uint16_t reading);

namespace frc971 {
namespace motors {

Motor::Motor(BigFTM *pwm_ftm, LittleFTM *encoder_ftm, MotorControls *controls,
             const ::std::array<volatile uint32_t *, 3> &output_registers)
    : pwm_ftm_(pwm_ftm),
      encoder_ftm_(encoder_ftm),
      controls_(controls),
      output_registers_(output_registers) {
  // PWMSYNC doesn't matter because we set SYNCMODE down below.
  pwm_ftm_->MODE = FTM_MODE_WPDIS;
  pwm_ftm_->MODE = FTM_MODE_WPDIS | FTM_MODE_FTMEN;
  encoder_ftm_->MODE = FTM_MODE_WPDIS;
  encoder_ftm_->MODE = FTM_MODE_WPDIS | FTM_MODE_FTMEN;

  pwm_ftm_->SC = FTM_SC_CLKS(0) /* Disable counting for now */;
  encoder_ftm_->SC =
      FTM_SC_CLKS(1) /* Use the system clock (not sure it matters) */ |
      FTM_SC_PS(0) /* Don't prescale the clock (not sure it matters) */;
}

static_assert((BUS_CLOCK_FREQUENCY % SWITCHING_FREQUENCY) == 0,
              "Switching frequency needs to divide the bus clock frequency");

static_assert(BUS_CLOCK_FREQUENCY / SWITCHING_FREQUENCY < UINT16_MAX,
              "Need to prescale");

void Motor::Init() {
  pwm_ftm_->CNTIN = encoder_ftm_->CNTIN = 0;
  pwm_ftm_->CNT = encoder_ftm_->CNT = 0;

  pwm_ftm_->MOD = counts_per_cycle() - 1;
  encoder_ftm_->MOD = controls_->mechanical_counts_per_revolution() - 1;

  // I think you have to set this to something other than 0 for the quadrature
  // encoder mode to actually work? This is "input capture on rising edge only",
  // which should be fine.
  encoder_ftm_->C0SC = FTM_CSC_ELSA;
  encoder_ftm_->C1SC = FTM_CSC_ELSA;

  // Initialize all the channels to 0.
  pwm_ftm_->OUTINIT = 0;

  // All of the channels are active high (low-side ones with separate high/low
  // drives are defaulted on elsewhere).
  pwm_ftm_->POL = 0;

  encoder_ftm_->FILTER = FTM_FILTER_CH0FVAL(0) /* No filter */ |
                         FTM_FILTER_CH1FVAL(0) /* No filter */;

  // Could set PHAFLTREN and PHBFLTREN here to enable the filters.
  encoder_ftm_->QDCTRL = FTM_QDCTRL_QUADEN;

  pwm_ftm_->SYNCONF =
      FTM_SYNCONF_HWWRBUF /* Hardware trigger flushes switching points */ |
      FTM_SYNCONF_SWWRBUF /* Software trigger flushes switching points */ |
      FTM_SYNCONF_SWRSTCNT /* Software trigger resets the count */ |
      FTM_SYNCONF_SYNCMODE /* Use the new synchronization mode */;
  encoder_ftm_->SYNCONF =
      FTM_SYNCONF_SWWRBUF /* Software trigger flushes MOD */ |
      FTM_SYNCONF_SWRSTCNT /* Software trigger resets the count */ |
      FTM_SYNCONF_SYNCMODE /* Use the new synchronization mode */;

  // Don't want any intermediate loading points.
  pwm_ftm_->PWMLOAD = 0;

  // This has to happen after messing with SYNCONF, and should happen after
  // messing with various other things so the values can get flushed out of the
  // buffers.
  pwm_ftm_->SYNC =
      FTM_SYNC_SWSYNC /* Flush everything out right now */ |
      FTM_SYNC_CNTMAX /* Load new values at the end of the cycle */;
  encoder_ftm_->SYNC = FTM_SYNC_SWSYNC /* Flush everything out right now */;

  // Wait for the software synchronization to finish.
  while (pwm_ftm_->SYNC & FTM_SYNC_SWSYNC) {
  }
  while (encoder_ftm_->SYNC & FTM_SYNC_SWSYNC) {
  }
}

void Motor::Start() {
  pwm_ftm_->SC = FTM_SC_TOIE /* Interrupt on overflow */ |
                 FTM_SC_CLKS(1) /* Use the system clock */ |
                 FTM_SC_PS(0) /* Don't prescale the clock */;
  pwm_ftm_->MODE &= ~FTM_MODE_WPDIS;
  encoder_ftm_->MODE &= ~FTM_MODE_WPDIS;
}

#define USE_ABSOLUTE_CUTOFF 0
#define DO_CONTROLS 1

#define USE_CUTOFF 1
#define PRINT_ALL_READINGS 0
#define TAKE_SAMPLE 0
#define SAMPLE_UNTIL_DONE 0
#define DO_STEP_RESPONSE 0
#define DO_PULSE_SWEEP 0
#define PRINT_TIMING 0

// An on-width of 60 with 30V in means about 50A through the motor and about
// 30W total power dumped by the motor for the big one.
// For the small one, an on-width of 120/3000 with 14V in means about 2A
// through the motor.
void Motor::CycleFixedPhaseInterupt(int period) {
  pwm_ftm_->SC &= ~FTM_SC_TOF;
  // Step through all the phases one by one in a loop.  This should slowly move
  // the trigger.
  // If we fire phase 1, we should go to PI radians.
  // If we fire phase 2, we should go to 1.0 * PI / 3.0 radians.
  // If we fire phase 3, we should go to -1.0 * PI / 3.0 radians.
  // These numbers were confirmed by the python motor simulation.
  static int phase_to_fire_count = -300000;
  static int phase_to_fire = 0;
  ++phase_to_fire_count;
  if (phase_to_fire_count > 500000) {
    phase_to_fire_count = 0;
    ++phase_to_fire;
    if (phase_to_fire > 2) {
      phase_to_fire = 0;
    }
  }
  phase_to_fire = 0;

  output_registers_[0][0] = 0;
  output_registers_[0][2] = phase_to_fire == 0 ? period : 0;

  const float switching_points_max = static_cast<float>(counts_per_cycle());
  switching_points_ratio_[0] =
      static_cast<float>(output_registers_[0][2]) / switching_points_max;
  output_registers_[1][0] = 0;
  output_registers_[1][2] = phase_to_fire == 1 ? period : 0;
  switching_points_ratio_[1] =
      static_cast<float>(output_registers_[1][2]) / switching_points_max;
  output_registers_[2][0] = 0;
  output_registers_[2][2] = phase_to_fire == 2 ? period : 0;
  switching_points_ratio_[2] =
      static_cast<float>(output_registers_[2][2]) / switching_points_max;

  // Tell the hardware to use the new switching points.
  // TODO(Brian): Somehow verify that we consistently hit the first or second
  // timer-cycle with the new values (if there's two).
  pwm_ftm_->PWMLOAD = FTM_PWMLOAD_LDOK;

  // If another cycle has already started, turn the light on right now.
  if (pwm_ftm_->SC & FTM_SC_TOF) {
    GPIOC_PSOR = 1 << 5;
  }
}

void Motor::CurrentInterrupt(const BalancedReadings &balanced,
                             uint32_t captured_wrapped_encoder) {
  pwm_ftm_->SC &= ~FTM_SC_TOF;

#if PRINT_TIMING
  const uint32_t start_nanos = nanos();
#endif  // PRINT_TIMING

  if (!time_after(time_add(last_current_set_time_, 100000), micros())) {
    goal_current_ = 0;
  }

#if DO_CONTROLS
  switching_points_ratio_ = controls_->DoIteration(
      balanced.readings, captured_wrapped_encoder, goal_current_);
  const float switching_points_max = static_cast<float>(counts_per_cycle());
  const ::std::array<uint32_t, 3> switching_points = {
      static_cast<uint32_t>(switching_points_ratio_[0] * switching_points_max),
      static_cast<uint32_t>(switching_points_ratio_[1] * switching_points_max),
      static_cast<uint32_t>(switching_points_ratio_[2] * switching_points_max)};
#if USE_CUTOFF
  constexpr uint32_t kMax = 2995;
  //constexpr uint32_t kMax = 1445;
  static bool done = false;
  bool done_now = false;
  if (switching_points[0] > kMax || switching_points[1] > kMax ||
      switching_points[2] > kMax) {
    done_now = true;
  }
#if USE_ABSOLUTE_CUTOFF
  static unsigned int current_done_count = 0;
  bool current_done = false;
  for (int phase = 0; phase < 3; ++phase) {
    const float scaled_reading =
        controls_->scale_current_reading(balanced.readings[phase]);
    static constexpr float kMaxBalancedCurrent = 50.0f;
    if (scaled_reading > kMaxBalancedCurrent ||
        scaled_reading < -kMaxBalancedCurrent) {
      current_done = true;
    }
  }
  if (current_done) {
    if (current_done_count > 5) {
      done_now = true;
    }
    ++current_done_count;
  } else {
    current_done_count = 0;
  }
#endif  // USE_ABSOLUTE_CUTOFF
  if (done_now && !done) {
    printf("done now\n");
    printf("switching_points %" PRIu32 " %" PRIu32 " %" PRIu32 "\n",
           switching_points[0], switching_points[1], switching_points[2]);
    printf("balanced %" PRIu16 " %" PRIu16 " %" PRIu16 "\n",
           static_cast<uint16_t>(balanced.readings[0]),
           static_cast<uint16_t>(balanced.readings[1]),
           static_cast<uint16_t>(balanced.readings[2]));
    done = true;
  }
  if (!done) {
#else  // USE_CUTOFF
  if (true) {
#endif  // USE_CUTOFF
    output_registers_[0][0] = CalculateOnTime(switching_points[0]);
    output_registers_[0][2] = CalculateOffTime(switching_points[0]);
    output_registers_[1][0] = CalculateOnTime(switching_points[1]);
    output_registers_[1][2] = CalculateOffTime(switching_points[1]);
    output_registers_[2][0] = CalculateOnTime(switching_points[2]);
    output_registers_[2][2] = CalculateOffTime(switching_points[2]);
    flip_time_offset_ = !flip_time_offset_;
  } else {
    output_registers_[0][0] = 0;
    output_registers_[0][2] = 0;
    output_registers_[1][0] = 0;
    output_registers_[1][2] = 0;
    output_registers_[2][0] = 0;
    output_registers_[2][2] = 0;
  }
#endif  // DO_CONTROLS
  (void)balanced;
  (void)captured_wrapped_encoder;
#if PRINT_ALL_READINGS
  printf("ref=%" PRIu16 " 0.0=%" PRIu16 " 1.0=%" PRIu16 " 2.0=%" PRIu16
         " in=%" PRIu16 " 0.1=%" PRIu16 " 1.1=%" PRIu16 " 2.1=%" PRIu16 "\n",
         adc_readings.motor_current_ref, adc_readings.motor_currents[0][0],
         adc_readings.motor_currents[1][0], adc_readings.motor_currents[2][0],
         adc_readings.input_voltage, adc_readings.motor_currents[0][1],
         adc_readings.motor_currents[1][1], adc_readings.motor_currents[2][1]);
#elif TAKE_SAMPLE  // PRINT_ALL_READINGS
#if 0
  constexpr int kStartupWait = 50000;
#elif 0
  constexpr int kStartupWait = 0;
#elif 0
  constexpr int kStartupWait = 30000;
#elif 1
  constexpr int kStartupWait = 2 * 20000;
#endif
  constexpr int kSubsampling = 1;
  //constexpr int kPoints = 5000;
  constexpr int kPoints = 1000;
  constexpr int kSamplingEnd = kStartupWait + kPoints * kSubsampling;
  (void)kSamplingEnd;
  static int j = 0;
  static int16_t data[kPoints][11];
  static int written = 0;
  static bool done_writing = false;
  static_assert((kStartupWait % kSubsampling) == 0, "foo");
  static_assert((kPoints % kSubsampling) == 0, "foo");
  if (j < kStartupWait) {
    // Wait to be started up.
    ++j;
#if SAMPLE_UNTIL_DONE
  } else if (!done) {
#else  // SAMPLE_UNTIL_DONE
  } else if (j < kSamplingEnd && (j % kSubsampling) == 0) {
#endif  // SAMPLE_UNTIL_DONE
    {
      const int index = ((j - kStartupWait) / kSubsampling) % kPoints;
      auto &point = data[index];
// Start obnoxious #if 0/#if 1
#if 0
      point[0] = adc_readings.motor_currents[0][0];
      point[1] = adc_readings.motor_currents[1][0];
      point[2] = adc_readings.motor_currents[2][0];
      point[3] = adc_readings.motor_currents[0][1];
      point[4] = adc_readings.motor_currents[1][1];
      point[5] = adc_readings.motor_currents[2][1];
#else
      point[0] = balanced.readings[0];
      point[1] = balanced.readings[1];
      point[2] = balanced.readings[2];
#if 1
      point[3] = controls_->Debug(0);
      point[4] = controls_->Debug(1);
      point[5] = controls_->Debug(2);
      point[6] = controls_->Debug(3);
      point[7] = controls_->Debug(4);
      point[8] = controls_->Debug(5);
      point[9] = controls_->Debug(6);
      point[10] = controls_->Debug(7);
#else
#if 0
      point[3] = adc_readings.motor_currents[0][0];
      point[4] = adc_readings.motor_currents[1][0];
      point[5] = adc_readings.motor_currents[2][0];
      point[6] = adc_readings.motor_currents[0][1];
      point[7] = adc_readings.motor_currents[1][1];
      point[8] = adc_readings.motor_currents[2][1];
#else
      point[3] = 0;
      point[4] = 0;
      point[5] = 0;
      point[6] = 0;
      point[7] = 0;
      point[8] = 0;
#endif
      point[9] = pwm_ftm_->C2V;
      point[10] = pwm_ftm_->C3V;
#endif
#if 0
      point[3] = pwm_ftm_->C1V - pwm_ftm_->C0V;
      point[4] = pwm_ftm_->C3V - pwm_ftm_->C2V;
      point[5] = pwm_ftm_->C5V - pwm_ftm_->C4V;
#endif
#endif
// End obnoxious #if 0/#if 1
      point[9] = captured_wrapped_encoder;
      //SmallInitReadings readings;
      //{
        //DisableInterrupts disable_interrupts;
        //readings = AdcReadSmallInit(disable_interrupts);
      //}
      //point[10] = readings.motor0_abs;
    }

#if DO_STEP_RESPONSE
    // Step response
    if (j > kStartupWait + 200 / kSubsampling) {
      pwm_ftm_->C3V = 240;
    }
#elif DO_PULSE_SWEEP  // DO_STEP_RESPONSE
    // Sweep the pulse through the ADC sampling points.
    static constexpr int kMax = 2500;
    static constexpr int kExtraWait = 1500;
    if (j > kStartupWait && j < kStartupWait + kExtraWait) {
      pwm_ftm_->C2V = 0;
      pwm_ftm_->C3V = 240;
    } else if (j < kStartupWait + kMax + kExtraWait) {
      uint32_t start = j - kStartupWait - kExtraWait;
      pwm_ftm_->C2V = start;
      pwm_ftm_->C3V = start + 240;
    } else {
      pwm_ftm_->C2V = 0;
      pwm_ftm_->C3V = 0;
    }
#endif  // DO_STEP_RESPONSE/DO_PULSE_SWEEP

    ++j;
#if SAMPLE_UNTIL_DONE
  } else if (false) {
#else  // SAMPLE_UNTIL_DONE
  } else if (j < kSamplingEnd) {
    ++j;
  } else if (j == kSamplingEnd) {
#endif  // SAMPLE_UNTIL_DONE
    printf("finished\n");
    ++j;
#if SAMPLE_UNTIL_DONE
  } else if (done) {
#else  // SAMPLE_UNTIL_DONE
  } else {
#endif  // SAMPLE_UNTIL_DONE
    // Time to write the data out.
    if (written < (int)sizeof(data) && printing_implementation_ != nullptr) {
      int to_write = sizeof(data) - written;
      if (to_write > 64) {
        to_write = 64;
      }
      int result = printing_implementation_->Write(((const char *)data) + written, to_write);
      if (result >= 0) {
        written += result;
      } else {
        printf("error\n");
      }
    }
#if 0
    if (!done_writing && written >= (int)sizeof(data) &&
        printing_implementation_->write_queue_empty()) {
      printf("done writing %d\n", written);
      done_writing = true;
    }
#endif
  }
#endif  // PRINT_ALL_READINGS/TAKE_SAMPLE
  (void)balanced;

  // Tell the hardware to use the new switching points.
  // TODO(Brian): Somehow verify that we consistently hit the first or second
  // timer-cycle with the new values (if there's two).
  pwm_ftm_->PWMLOAD = FTM_PWMLOAD_LDOK;

#if PRINT_TIMING
  static int print_timing_count = 0;
  static uint32_t print_timing_total = 0;
  print_timing_total += time_subtract(nanos(), start_nanos);
  if (++print_timing_count == 1000) {
    printf("took %" PRIu32 "/%d\n", print_timing_total, print_timing_count);
    print_timing_count = 0;
    print_timing_total = 0;
  }
#endif  // PRINT_TIMING

  // If another cycle has already started, turn the light on right now.
  if (pwm_ftm_->SC & FTM_SC_TOF) {
    GPIOC_PSOR = 1 << 5;
  }
}

uint32_t Motor::CalculateOnTime(uint32_t width) const {
  if (width > 0) {
    width += deadtime_compensation_;
    if (flip_time_offset_) {
      width += 1;
    }
  }
  return (counts_per_cycle() - width) / 2;
}

uint32_t Motor::CalculateOffTime(uint32_t width) const {
  if (width > 0) {
    width += deadtime_compensation_;
    if (!flip_time_offset_) {
      width += 1;
    }
  }
  return (counts_per_cycle() + width) / 2;
}

}  // namespace motors
}  // namespace frc971
