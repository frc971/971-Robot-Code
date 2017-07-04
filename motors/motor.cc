#include "motors/motor.h"

#include <limits.h>

#include <array>

#include "motors/peripheral/configuration.h"

#include <stdio.h>
#include <inttypes.h>
#include "motors/core/time.h"
#include "motors/usb/usb_serial.h"
#include "motors/peripheral/can.h"

namespace frc971 {
namespace salsa {
namespace {

constexpr int kDeadtimeCompensation = 9;

uint32_t CalculateOnTime(uint32_t width, bool flip) {
  if (width > 0) {
    width += kDeadtimeCompensation;
    if (flip) {
      width += 1;
    }
  }
  return 1500 - width / 2;
}

uint32_t CalculateOffTime(uint32_t width, bool flip) {
  if (width > 0) {
    width += kDeadtimeCompensation;
    if (!flip) {
      width += 1;
    }
  }
  return 1500 + width / 2;
}

}  // namespace

Motor::Motor(BigFTM *pwm_ftm, LittleFTM *encoder_ftm, MotorControls *controls)
    : pwm_ftm_(pwm_ftm), encoder_ftm_(encoder_ftm), controls_(controls) {}

static_assert((BUS_CLOCK_FREQUENCY % SWITCHING_FREQUENCY) == 0,
              "Switching frequency needs to divide the bus clock frequency");

static_assert(BUS_CLOCK_FREQUENCY / SWITCHING_FREQUENCY < UINT16_MAX,
              "Need to prescale");

void Motor::Init() {
  PORTE_PCR24 = PORT_PCR_MUX(1);
  PORTB_PCR0 = PORT_PCR_MUX(6);
  PORTB_PCR1 = PORT_PCR_MUX(6);

  PORTC_PCR1 = PORT_PCR_MUX(4);
  PORTC_PCR2 = PORT_PCR_MUX(4);
  PORTC_PCR3 = PORT_PCR_MUX(4);
  PORTC_PCR4 = PORT_PCR_MUX(4);
  PORTD_PCR4 = PORT_PCR_MUX(4);
  PORTD_PCR5 = PORT_PCR_MUX(4);

  // PWMSYNC doesn't matter because we set SYNCMODE down below.
  pwm_ftm_->MODE = FTM_MODE_WPDIS;
  pwm_ftm_->MODE = FTM_MODE_WPDIS | FTM_MODE_FTMEN;
  encoder_ftm_->MODE = FTM_MODE_WPDIS;
  encoder_ftm_->MODE = FTM_MODE_WPDIS | FTM_MODE_FTMEN;

  pwm_ftm_->SC = FTM_SC_CLKS(0) /* Disable counting for now */;
  encoder_ftm_->SC =
      FTM_SC_CLKS(1) /* Use the system clock (not sure it matters) */ |
      FTM_SC_PS(0) /* Don't prescale the clock (not sure it matters) */;

  pwm_ftm_->CNTIN = encoder_ftm_->CNTIN = 0;
  pwm_ftm_->CNT = encoder_ftm_->CNT = 0;

  pwm_ftm_->MOD = (BUS_CLOCK_FREQUENCY / SWITCHING_FREQUENCY) - 1;
  encoder_ftm_->MOD = controls_->counts_per_revolution() - 1;

  // Put them all into combine active-high mode, and all the low ones staying on
  // all the time by default.
  // TODO: update comment
  pwm_ftm_->C0SC = FTM_CSC_ELSA;
  pwm_ftm_->C0V = 0;
  pwm_ftm_->C1SC = FTM_CSC_ELSA;
  pwm_ftm_->C1V = 0;
  pwm_ftm_->C2SC = FTM_CSC_ELSA;
  pwm_ftm_->C2V = 0;
  pwm_ftm_->C3SC = FTM_CSC_ELSA;
  pwm_ftm_->C3V = 0;
  pwm_ftm_->C4SC = FTM_CSC_ELSA;
  pwm_ftm_->C4V = 0;
  pwm_ftm_->C5SC = FTM_CSC_ELSA;
  pwm_ftm_->C5V = 0;

  // I think you have to set this to something other than 0 for the quadrature
  // encoder mode to actually work? This is "input capture on rising edge only",
  // which should be fine.
  encoder_ftm_->C0SC = FTM_CSC_ELSA;
  encoder_ftm_->C1SC = FTM_CSC_ELSA;

  // Initialize all the channels to 0.
  // TODO(Brian): Is this really what we want?
  pwm_ftm_->OUTINIT = 0;

  pwm_ftm_->COMBINE = FTM_COMBINE_SYNCEN3 /* Synchronize updates usefully */ |
                      FTM_COMBINE_DTEN3 /* Enable deadtime */ |
                      FTM_COMBINE_COMP3 /* Make them complementary */ |
                      FTM_COMBINE_COMBINE3 /* Combine the channels */ |
                      FTM_COMBINE_SYNCEN2 /* Synchronize updates usefully */ |
                      FTM_COMBINE_DTEN2 /* Enable deadtime */ |
                      FTM_COMBINE_COMP2 /* Make them complementary */ |
                      FTM_COMBINE_COMBINE2 /* Combine the channels */ |
                      FTM_COMBINE_SYNCEN1 /* Synchronize updates usefully */ |
                      FTM_COMBINE_DTEN1 /* Enable deadtime */ |
                      FTM_COMBINE_COMP1 /* Make them complementary */ |
                      FTM_COMBINE_COMBINE1 /* Combine the channels */ |
                      FTM_COMBINE_SYNCEN0 /* Synchronize updates usefully */ |
                      FTM_COMBINE_DTEN0 /* Enable deadtime */ |
                      FTM_COMBINE_COMP0 /* Make them complementary */ |
                      FTM_COMBINE_COMBINE0 /* Combine the channels */;

  // Set a deadtime of 500ns.
  // TODO: update comment
  pwm_ftm_->DEADTIME =
      FTM_DEADTIME_DTPS(0) /* Prescaler of 1 */ | FTM_DEADTIME_DTVAL(9);

  // All of the channels are active high.
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

void Motor::Zero() {
#if 0
  while (true) {
    if (GPIO_BITBAND(GPIOE_PDIR, 24)) {
      encoder_ftm_->CNT = 0;
      break;
    }
  }
#else
  uint32_t scratch;
  __disable_irq();
  // Stuff all of this in an inline assembly statement so we can make sure the
  // compiler doesn't decide sticking constant loads etc in the middle of
  // the loop is a good idea, because that increases the latency of recognizing
  // the index pulse edge which makes velocity affect the zeroing accuracy.
  __asm__ __volatile__(
      // A label to restart the loop.
      "0:\n"
      // Load the current PDIR value for the pin we care about.
      "ldr %[scratch], [%[pdir_word]]\n"
      // Terminate the loop if it's non-0.
      "cbnz %[scratch], 1f\n"
      // Go back around again.
      "b 0b\n"
      // A label to finish the loop.
      "1:\n"
      // Reset the count once we're down here. It doesn't actually matter what
      // value we store because writing anything resets it to CNTIN (ie 0).
      "str %[scratch], [%[cnt]]\n"
      : [scratch] "=&l"(scratch)
      : [pdir_word] "l"(&GPIO_BITBAND(GPIOE_PDIR, 24)),
        [cnt] "l"(&encoder_ftm_->CNT));
  __enable_irq();
#endif
}

void Motor::Start() {
  pwm_ftm_->SC = FTM_SC_TOIE /* Interrupt on overflow */ |
                 FTM_SC_CLKS(1) /* Use the system clock */ |
                 FTM_SC_PS(0) /* Don't prescale the clock */;
  pwm_ftm_->MODE &= ~FTM_MODE_WPDIS;
  encoder_ftm_->MODE &= ~FTM_MODE_WPDIS;

  NVIC_ENABLE_IRQ(IRQ_FTM0);
}

void Motor::HandleInterrupt() {
  const uint32_t starting_sc = pwm_ftm_->SC;
  pwm_ftm_->SC = starting_sc & ~FTM_SC_TOF;

  const uint32_t start_count = pwm_ftm_->CNT;
  __asm__("":::"memory");
  const MediumAdcReadings adc_readings = AdcReadMedium();

  // Turn the light on if we're starting too late (this generally means a
  // previous iteration took too long).
  if (start_count > 100) {
    GPIOC_PSOR = 1 << 5;
  }

  ReadingsToBalance to_balance{{0, 0, 0}, {0, 0, 0}};
  {
    for (int reading = 0; reading < 2; ++reading) {
      for (int phase = 0; phase < 3; ++phase) {
        to_balance.Add(phase, adc_readings.motor_currents[phase][reading]);
      }
    }
  }
  const BalancedReadings balanced = BalanceReadings(to_balance);

  static float current_command = 0;
  static uint32_t last_command_receive_time = 0;
  {
    unsigned char command_data[8];
    int command_length;
    can_receive_command(command_data, &command_length);
    if (command_length == 4) {
      last_command_receive_time = micros();
      uint32_t result = command_data[0] << 24 | command_data[1] << 16 |
                        command_data[2] << 8 | command_data[3];
      current_command = static_cast<float>(result) / 1000.0f;
    }
  }
  if (!time_after(time_add(last_command_receive_time, 100000), micros())) {
    current_command = 0;
  }

  static bool high_gear = false;
  if (controls_->estimated_velocity() < -2015) {
    high_gear = true;
  }
  if (current_command < 1) {
    high_gear = false;
  }
  float current_now = current_command;
  if (!high_gear) {
    current_now = current_now * -120.0f / 120.0f;
  } else {
    current_now = current_now * 115.0f / 120.0f;
  }

#if 0
  static int status_send_counter = 0;
  if (++status_send_counter == 1000) {
    // can_send(uint32_t can_id, const unsigned char *data, unsigned int length)
    unsigned char send_data[8];
    can_send(9 << 8, send_data, 0);
    status_send_counter = 0;
    printf("sent\n");
  }
#endif

#define DO_CONTROLS 1
#if DO_CONTROLS
  static constexpr int kEncoderOffset = 810;
  const uint32_t adjusted_count = (encoder_ftm_->CNT + kEncoderOffset) % 1024;
  const ::std::array<uint32_t, 3> switching_points =
      controls_->DoIteration(balanced.readings, adjusted_count, current_now);
  constexpr uint32_t kMax = 2945;
  static bool done = false;
  bool done_now = false;
  if (switching_points[0] > kMax || switching_points[1] > kMax ||
      switching_points[2] > kMax) {
    done_now = true;
  }
#define USE_ABSOLUTE_CUTOFF 1
#if USE_ABSOLUTE_CUTOFF
  static unsigned int current_done_count = 0;
  bool current_done = false;
  for (int phase = 0; phase < 3; ++phase) {
    const float scaled_reading =
        MotorControls::scale_current_reading(balanced.readings[0]);
    static constexpr float kMaxBalancedCurrent = 190.0f;
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
#endif
  if (done_now && !done) {
    printf("done now\n");
    printf("switching_points %" PRIu32 " %" PRIu32 " %" PRIu32 "\n",
           switching_points[0], switching_points[1], switching_points[2]);
    printf("raw %" PRIu16 " %" PRIu16 " %" PRIu16 " %" PRIu16 " %" PRIu16
           " %" PRIu16 "\n",
           adc_readings.motor_currents[0][0], adc_readings.motor_currents[0][1],
           adc_readings.motor_currents[1][0], adc_readings.motor_currents[1][1],
           adc_readings.motor_currents[2][0],
           adc_readings.motor_currents[2][1]);
    printf("balanced %" PRIu16 " %" PRIu16 " %" PRIu16 "\n",
           static_cast<uint16_t>(balanced.readings[0]),
           static_cast<uint16_t>(balanced.readings[1]),
           static_cast<uint16_t>(balanced.readings[2]));
    done = true;
  }
  if (!done) {
    pwm_ftm_->C0V = CalculateOnTime(switching_points[0], flip_time_offset_);
    pwm_ftm_->C1V = CalculateOffTime(switching_points[0], flip_time_offset_);
    pwm_ftm_->C2V = CalculateOnTime(switching_points[1], flip_time_offset_);
    pwm_ftm_->C3V = CalculateOffTime(switching_points[1], flip_time_offset_);
    pwm_ftm_->C4V = CalculateOnTime(switching_points[2], flip_time_offset_);
    pwm_ftm_->C5V = CalculateOffTime(switching_points[2], flip_time_offset_);
    flip_time_offset_ = !flip_time_offset_;
  } else {
    pwm_ftm_->C0V = 0;
    pwm_ftm_->C1V = 0;
    pwm_ftm_->C2V = 0;
    pwm_ftm_->C3V = 0;
    pwm_ftm_->C4V = 0;
    pwm_ftm_->C5V = 0;
  }
#define DO_FIXED_PULSE 0
#elif DO_FIXED_PULSE
  // An on-width of 60 with 30V in means about 50A through the motor and about
  // 30W total power dumped by the motor.
#if 0
  static int i = 0;
  ++i;
  if (i == 2) {
    pwm_ftm_->C3V = 111;
    i = 0;
  } else {
    pwm_ftm_->C3V = 0;
  }
#endif
  pwm_ftm_->C0V = 0;
  pwm_ftm_->C1V = 0;
  pwm_ftm_->C2V = 0;
  //pwm_ftm_->C3V = 100;
  pwm_ftm_->C4V = 0;
  pwm_ftm_->C5V = 0;
#endif

#define PRINT_READINGS 0
#if PRINT_READINGS
  static int i = 0;
  if (i == 100) {
    i = 0;
    printf("i=%" PRIu16 "\n", adc_readings.input_voltage);
  } else if (i == 20) {
    printf("0=%" PRIu16 " r=%" PRIu16 "\n",
           adc_readings.motor_currents[0][0], adc_readings.motor_current_ref);
  } else if (i == 40) {
    printf("1=%" PRIu16 " r=%" PRIu16 "\n",
           adc_readings.motor_currents[1][0], adc_readings.motor_current_ref);
  } else if (i == 60) {
    printf("2=%" PRIu16 " r=%" PRIu16 "\n",
           adc_readings.motor_currents[2][0], adc_readings.motor_current_ref);
  } else {
    //printf("%" PRIu32 " to %" PRIu32 "\n", start_count, end_count);
  }
  ++i;
#define PRINT_ALL_READINGS 0
#elif PRINT_ALL_READINGS
  printf("ref=%" PRIu16 " 0.0=%" PRIu16 " 1.0=%" PRIu16 " 2.0=%" PRIu16
         " in=%" PRIu16 " 0.1=%" PRIu16 " 1.1=%" PRIu16 " 2.1=%" PRIu16 "\n",
         adc_readings.motor_current_ref, adc_readings.motor_currents[0][0],
         adc_readings.motor_currents[1][0], adc_readings.motor_currents[2][0],
         adc_readings.input_voltage, adc_readings.motor_currents[0][1],
         adc_readings.motor_currents[1][1], adc_readings.motor_currents[2][1]);
#define TAKE_SAMPLE 1
#elif TAKE_SAMPLE
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
  constexpr int kPoints = 5000;
  constexpr int kSamplingEnd = kStartupWait + kPoints * kSubsampling;
  (void)kSamplingEnd;
  static int j = 0;
  static int16_t data[kPoints][11];
  static int written = 0;
  static_assert((kStartupWait % kSubsampling) == 0, "foo");
  static_assert((kPoints % kSubsampling) == 0, "foo");
  if (j < kStartupWait) {
    // Wait to be started up.
    ++j;
#define SAMPLE_UNTIL_DONE 0
#if !SAMPLE_UNTIL_DONE
  } else if (j < kSamplingEnd && (j % kSubsampling) == 0) {
#else
  } else if (!done) {
#endif
    {
      const int index = ((j - kStartupWait) / kSubsampling) % kPoints;
      auto point = data[index];
#if 0
      point[0] = adc_readings.motor_currents[0][0];
      point[1] = adc_readings.motor_currents[1][0];
      point[2] = adc_readings.motor_currents[2][0];
      point[3] = adc_readings.motor_currents[0][1];
      point[4] = adc_readings.motor_currents[1][1];
      point[5] = adc_readings.motor_currents[2][1];
#else
      point[0] = balanced.readings[0] / 2;
      point[1] = balanced.readings[1] / 2;
      point[2] = balanced.readings[2] / 2;
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
      point[3] = adc_readings.motor_currents[0][0];
      point[4] = adc_readings.motor_currents[1][0];
      point[5] = adc_readings.motor_currents[2][0];
      point[6] = adc_readings.motor_currents[0][1];
      point[7] = adc_readings.motor_currents[1][1];
      point[8] = adc_readings.motor_currents[2][1];
      point[9] = temp1;
      point[10] = temp2;
#endif
      (void)temp1;
      (void)temp2;
#if 0
      point[3] = pwm_ftm_->C1V - pwm_ftm_->C0V;
      point[4] = pwm_ftm_->C3V - pwm_ftm_->C2V;
      point[5] = pwm_ftm_->C5V - pwm_ftm_->C4V;
#endif
#endif
#if 0
      point[6] = adc_readings.motor_current_ref;
      point[7] = adc_readings.input_voltage;
#else
#endif
    }

#define DO_STEP_RESPONSE 0
#if DO_STEP_RESPONSE
    // Step response
    if (j > 25000) {
      pwm_ftm_->C1V = 20;
    }
#define DO_PULSE_SWEEP 0
#elif DO_PULSE_SWEEP
    // Sweep the pulse through the ADC sampling points.
    static constexpr int kMax = 2500;
    static constexpr int kExtraWait = 1500;
    if (j > kStartupWait && j < kStartupWait + kExtraWait) {
      pwm_ftm_->C4V = 0;
      pwm_ftm_->C5V = 60;
    } else if (j < kStartupWait + kMax + kExtraWait) {
      uint32_t start = j - kStartupWait - kExtraWait;
      pwm_ftm_->C4V = start;
      pwm_ftm_->C5V = start + 60;
    } else {
      pwm_ftm_->C4V = 0;
      pwm_ftm_->C5V = 0;
    }
#endif

    ++j;
#if !SAMPLE_UNTIL_DONE
  } else if (j < kSamplingEnd) {
    ++j;
  } else if (j == kSamplingEnd) {
#else
  } else if (false) {
#endif
    printf("finished\n");
    ++j;
#if !SAMPLE_UNTIL_DONE
  } else {
#else
  } else if (done) {
#endif
    // Time to write the data out.
    if (written < (int)sizeof(data)) {
      int to_write = sizeof(data) - written;
#if 1
      if (to_write > 20) {
        to_write = 20;
      }
      // TODO(Brian): Fix usb_serial_write so we can write more than 1 byte at a
      // time.
      if (to_write > 1) {
        to_write = 1;
      }
      int result =
          usb_serial_write(1, ((const char *)data) + written, to_write);
#else
      if (to_write > 8) {
        to_write = 8;
      }
      int result =
          can_send(97, ((const unsigned char *)data) + written, to_write);
#endif
      if (result >= 0) {
        written += to_write;
      } else {
        //printf("error\n");
      }
      if (written == (int)sizeof(data)) {
        printf("done writing %d\n", written);
      }
    }
  }
#endif
  (void)adc_readings;
  (void)start_count;
  (void)balanced;

  // Tell the hardware to use the new switching points.
  pwm_ftm_->PWMLOAD = FTM_PWMLOAD_LDOK;

  // If another cycle has already started, turn the light on right now.
  if (pwm_ftm_->SC & FTM_SC_TOF) {
    GPIOC_PSOR = 1 << 5;
  }
}

}  // namespace salsa
}  // namespace frc971
