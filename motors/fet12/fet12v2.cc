#include "motors/core/kinetis.h"

#include <inttypes.h>
#include <stdio.h>

#include <atomic>

#include "motors/core/time.h"
#include "motors/fet12/current_equalization.h"
#include "motors/fet12/motor_controls.h"
#include "motors/motor.h"
#include "motors/peripheral/adc.h"
#include "motors/peripheral/adc_dma.h"
#include "motors/peripheral/can.h"
#include "motors/print/print.h"
#include "motors/util.h"

namespace frc971 {
namespace motors {
namespace {

constexpr double Kv = 22000.0 * 2.0 * M_PI / 60.0 / 30.0 * 3.6;
constexpr double kVcc = 31.5;
constexpr double kIcc = 125.0;
constexpr double kR = 0.0084;

struct Fet12AdcReadings {
  // Averages of the pairs of ADC DMA channels corresponding with each channel
  // pair. Individual values in motor_currents correspond to current sensor
  // values, rather than the actual currents themselves (and so they still need
  // to be decoupled).
  int16_t motor_currents[3];
  int16_t throttle, fuse_voltage;
};

void AdcInitFet12() {
  AdcInitCommon(AdcChannels::kB, AdcChannels::kA);

  // M_CH0V ADC0_SE5b
  PORTD_PCR1 = PORT_PCR_MUX(0);

  // M_CH1V ADC0_SE7b
  PORTD_PCR6 = PORT_PCR_MUX(0);

  // M_CH2V ADC0_SE14
  PORTC_PCR0 = PORT_PCR_MUX(0);

  // M_CH0F ADC1_SE5a
  PORTE_PCR1 = PORT_PCR_MUX(0);

  // M_CH1F ADC1_SE6a
  PORTE_PCR2 = PORT_PCR_MUX(0);

  // M_CH2F ADC1_SE7a
  PORTE_PCR3 = PORT_PCR_MUX(0);

  // SENSE0 ADC0_SE23
  // dedicated

  // SENSE1 ADC0_SE13
  PORTB_PCR3 = PORT_PCR_MUX(0);
}

::std::atomic<Motor *> global_motor{nullptr};
::std::atomic<teensy::AdcDmaSampler *> global_adc_dma{nullptr};

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

extern char *__brkval;
extern uint32_t __bss_ram_start__[];
extern uint32_t __heap_start__[];
extern uint32_t __stack_end__[];

struct DebugBuffer {
  struct Sample {
    ::std::array<int16_t, 3> currents;
    ::std::array<int16_t, 3> commanded_currents;
    ::std::array<uint16_t, 3> commands;
    ::std::array<int16_t, 3> readings;
    uint16_t position;
    // Driver requested current.
    float driver_request;
    // Requested current.
    int16_t total_command;

    float est_omega;
    float fuse_voltage;
    int16_t fuse_current;

    float fuse_badness;
    uint32_t cycles_since_start;
  };

  // The amount of data in the buffer.  This will never decrement.  This will be
  // transferred out the serial port after it fills up.
  ::std::atomic<size_t> size{0};
  ::std::atomic<uint32_t> count{0};
  // The data.
  ::std::array<Sample, 512> samples;
};

DebugBuffer global_debug_buffer;

void ftm0_isr(void) {
  static uint32_t i = 0;
  teensy::AdcDmaSampler *const adc_dma =
      global_adc_dma.load(::std::memory_order_relaxed);

  Fet12AdcReadings adc_readings;
  // TODO(Brian): Switch to the DMA interrupt instead of spinning.
  while (!adc_dma->CheckDone()) {
  }

  adc_readings.motor_currents[0] =
      (adc_dma->adc_result(0, 0) + adc_dma->adc_result(0, 1)) / 2;
  adc_readings.motor_currents[1] =
      (adc_dma->adc_result(0, 2) + adc_dma->adc_result(1, 1)) / 2;
  adc_readings.motor_currents[2] =
      (adc_dma->adc_result(1, 0) + adc_dma->adc_result(1, 2)) / 2;
  adc_readings.throttle = adc_dma->adc_result(0, 3);
  const ::std::array<float, 3> decoupled =
      DecoupleCurrents(adc_readings.motor_currents);
  adc_dma->Reset();
  const uint32_t wrapped_encoder =
      global_motor.load(::std::memory_order_relaxed)->wrapped_encoder();
  const BalancedReadings balanced =
      BalanceSimpleReadings(decoupled);

#if 1

  static float fuse_badness = 0;

  static uint32_t cycles_since_start = 0u;
  ++cycles_since_start;
#if 0
  static int count = 0;
  ++count;
  static float currents[3] = {0.0f, 0.0f, 0.0f};
  for (int ii = 0; ii < 3; ++ii) {
    currents[ii] += static_cast<float>(adc_readings.motor_currents[ii]);
  }

  if (i == 0) {
    printf(
        "foo %d.0, %d.0, %d.0, %.3d %.3d %.3d, switching %d %d %d enc %d\n",
        static_cast<int>(currents[0] / static_cast<float>(count)),
        static_cast<int>(currents[1] / static_cast<float>(count)),
        static_cast<int>(currents[2] / static_cast<float>(count)),
        static_cast<int>(decoupled[0] * 1.0f),
        static_cast<int>(decoupled[1] * 1.0f),
        static_cast<int>(decoupled[2] * 1.0f),
        global_motor.load(::std::memory_order_relaxed)->get_switching_points_cycles(0),
        global_motor.load(::std::memory_order_relaxed)->get_switching_points_cycles(1),
        global_motor.load(::std::memory_order_relaxed)->get_switching_points_cycles(2),
        static_cast<int>(
            global_motor.load(::std::memory_order_relaxed)->wrapped_encoder()));
    count = 0;
    currents[0] = 0.0f;
    currents[1] = 0.0f;
    currents[2] = 0.0f;
  }
#endif
#if 1
  constexpr float kAlpha = 0.995f;
  constexpr float kFuseAlpha = 0.95f;

  // 3400 - 760
  // Start the throttle filter at 1.0f--once it converges to near zero, we set
  // throttle_zeroed to true and only then do we start listening to throttle
  // commands.
  static float filtered_throttle = 1.0f;
  static bool throttle_zeroed = false;
  constexpr int kMaxThrottle = 3400;
  constexpr int kMinThrottle = 760;
  const float throttle = ::std::max(
      0.0f,
      ::std::min(1.0f,
                 static_cast<float>(static_cast<int>(adc_readings.throttle) -
                                    kMinThrottle) /
                     static_cast<float>(kMaxThrottle - kMinThrottle)));

  // y(n) = x(n) + a * (y(n-1) - x(n))
  filtered_throttle = throttle + kAlpha * (filtered_throttle - throttle);
  if (::std::abs(filtered_throttle) < 1e-2f) {
    // Once the filter gets near zero once, we start paying attention to it;
    // once it gets near zero once, never ignore it again.
    throttle_zeroed = true;
  }

  const float fuse_voltage = static_cast<float>(adc_readings.fuse_voltage);
  static float filtered_fuse_voltage = 0.0f;

  filtered_fuse_voltage =
      fuse_voltage + kFuseAlpha * (filtered_fuse_voltage - fuse_voltage);

  const float velocity =
      global_motor.load(::std::memory_order_relaxed)->estimated_velocity();
  const float bemf = velocity / (static_cast<float>(Kv) / 1.5f);
  const float abs_bemf = ::std::abs(bemf);
  constexpr float kPeakCurrent = 300.0f;
  constexpr float kLimitedCurrent = 70.0f;
  const float max_bat_cur =
      fuse_badness > (kLimitedCurrent * kLimitedCurrent * 0.95f)
          ? kLimitedCurrent
          : static_cast<float>(kIcc);
  const float throttle_limit = ::std::min(
      kPeakCurrent,
      (-abs_bemf + ::std::sqrt(static_cast<float>(
                       bemf * bemf +
                       4.0f * static_cast<float>(kR) * 1.5f *
                           static_cast<float>(kVcc) * max_bat_cur))) /
          (2.0f * 1.5f * static_cast<float>(kR)));

  constexpr float kNegativeCurrent = 100.0f;
  float goal_current =
      -::std::min(
          ::std::max(filtered_throttle * (kPeakCurrent + kNegativeCurrent) -
                         kNegativeCurrent,
                     -throttle_limit),
          throttle_limit);

  if (!throttle_zeroed) {
    goal_current = 0.0f;
  }
  // Note: current reduction is 12/70 belt, 15 / 54 on chain, and 10 inch
  // diameter wheels, so cutoff of 500 electrical rad/sec * 1 mechanical rad / 2
  // erad * 12 / 70 * 15 / 54 * 0.127 m = 1.5m/s = 3.4 mph
  if (velocity > -500) {
    if (goal_current > 0.0f) {
      goal_current = 0.0f;
    }
  }

  //float goal_current =
      //-::std::min(filtered_throttle * kPeakCurrent, throttle_limit);
  const float overall_measured_current =
      global_motor.load(::std::memory_order_relaxed)
          ->overall_measured_current();
  const float fuse_current =
      overall_measured_current *
      (bemf + overall_measured_current * static_cast<float>(kR) * 1.5f) /
      static_cast<float>(kVcc);
  const int16_t fuse_current_10 = static_cast<int16_t>(10.0f * fuse_current);
  fuse_badness += 0.00002f * (fuse_current * fuse_current - fuse_badness);

  global_motor.load(::std::memory_order_relaxed)
      ->SetGoalCurrent(goal_current);
  global_motor.load(::std::memory_order_relaxed)
      ->CurrentInterrupt(balanced, wrapped_encoder);

  global_debug_buffer.count.fetch_add(1);

  const bool trigger = false && i > 10000;
  // global_debug_buffer.count.load(::std::memory_order_relaxed) >= 0;
  size_t buffer_size =
      global_debug_buffer.size.load(::std::memory_order_relaxed);
  if ((buffer_size > 0 || trigger) &&
      buffer_size != global_debug_buffer.samples.size()) {
    global_debug_buffer.samples[buffer_size].currents[0] =
        static_cast<int16_t>(balanced.readings[0] * 10.0f);
    global_debug_buffer.samples[buffer_size].currents[1] =
        static_cast<int16_t>(balanced.readings[1] * 10.0f);
    global_debug_buffer.samples[buffer_size].currents[2] =
        static_cast<int16_t>(balanced.readings[2] * 10.0f);
    global_debug_buffer.samples[buffer_size].position =
        global_motor.load(::std::memory_order_relaxed)->wrapped_encoder();
    global_debug_buffer.samples[buffer_size].est_omega =
        global_motor.load(::std::memory_order_relaxed)->estimated_velocity();
    global_debug_buffer.samples[buffer_size].commands[0] =
        global_motor.load(::std::memory_order_relaxed)->get_switching_points_cycles(0);
    global_debug_buffer.samples[buffer_size].commands[1] =
        global_motor.load(::std::memory_order_relaxed)->get_switching_points_cycles(1);
    global_debug_buffer.samples[buffer_size].commands[2] =
        global_motor.load(::std::memory_order_relaxed)->get_switching_points_cycles(2);
    global_debug_buffer.samples[buffer_size].commanded_currents[0] =
        global_motor.load(::std::memory_order_relaxed)->i_goal(0);
    global_debug_buffer.samples[buffer_size].commanded_currents[1] =
        global_motor.load(::std::memory_order_relaxed)->i_goal(1);
    global_debug_buffer.samples[buffer_size].commanded_currents[2] =
        global_motor.load(::std::memory_order_relaxed)->i_goal(2);
    global_debug_buffer.samples[buffer_size].total_command =
        global_motor.load(::std::memory_order_relaxed)->goal_current();
    global_debug_buffer.samples[buffer_size].fuse_voltage =
        filtered_fuse_voltage;
    global_debug_buffer.samples[buffer_size].fuse_current = fuse_current_10;
    global_debug_buffer.samples[buffer_size].driver_request =
        ::std::max(filtered_throttle * (kPeakCurrent + kNegativeCurrent) -
                       kNegativeCurrent,
                   0.0f);
    global_debug_buffer.samples[buffer_size].fuse_badness = fuse_badness;
    global_debug_buffer.samples[buffer_size].cycles_since_start = cycles_since_start;

    global_debug_buffer.size.fetch_add(1);
  }

  ++i;
  if (buffer_size == global_debug_buffer.samples.size()) {
    i = 0;
    GPIOC_PCOR = (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4);
    GPIOD_PCOR = (1 << 4) | (1 << 5);

    PERIPHERAL_BITBAND(GPIOC_PDDR, 1) = 1;
    PERIPHERAL_BITBAND(GPIOC_PDDR, 2) = 1;
    PERIPHERAL_BITBAND(GPIOC_PDDR, 3) = 1;
    PERIPHERAL_BITBAND(GPIOC_PDDR, 4) = 1;
    PERIPHERAL_BITBAND(GPIOD_PDDR, 4) = 1;
    PERIPHERAL_BITBAND(GPIOD_PDDR, 5) = 1;

    PORTC_PCR1 = PORT_PCR_DSE | PORT_PCR_MUX(1);
    PORTC_PCR2 = PORT_PCR_DSE | PORT_PCR_MUX(1);
    PORTC_PCR3 = PORT_PCR_DSE | PORT_PCR_MUX(1);
    PORTC_PCR4 = PORT_PCR_DSE | PORT_PCR_MUX(1);
    PORTD_PCR4 = PORT_PCR_DSE | PORT_PCR_MUX(1);
    PORTD_PCR5 = PORT_PCR_DSE | PORT_PCR_MUX(1);
  }
#endif
#else
  // Useful code when calculating resistance/inductance of motor
  FTM0->SC &= ~FTM_SC_TOF;
  FTM0->C0V = 0;
  FTM0->C1V = 0;
  FTM0->C2V = 0;
  FTM0->C3V = 20;
  FTM0->C4V = 0;
  FTM0->C5V = 0;
  FTM0->PWMLOAD = FTM_PWMLOAD_LDOK;
  (void)wrapped_encoder;
  size_t buffer_size =
      global_debug_buffer.size.load(::std::memory_order_relaxed);
  // Setting this to true is helpful for calibrating inductance, and false is
  // good for calibrating resistance.
  constexpr bool start_immediately = true;
  bool trigger = start_immediately || i > 20000;
  if ((trigger || buffer_size > 0) &&
      buffer_size != global_debug_buffer.samples.size()) {
    global_debug_buffer.samples[buffer_size].currents[0] =
        static_cast<int16_t>(balanced.readings[0] * 10.0f);
    global_debug_buffer.samples[buffer_size].currents[1] =
        static_cast<int16_t>(balanced.readings[1] * 10.0f);
    global_debug_buffer.samples[buffer_size].currents[2] =
        static_cast<int16_t>(balanced.readings[2] * 10.0f);
    global_debug_buffer.samples[buffer_size].commands[0] = FTM0->C1V;
    global_debug_buffer.samples[buffer_size].commands[1] = FTM0->C3V;
    global_debug_buffer.samples[buffer_size].commands[2] = FTM0->C5V;
    global_debug_buffer.samples[buffer_size].readings[0] =
        adc_readings.motor_currents[0];
    global_debug_buffer.samples[buffer_size].readings[1] =
        adc_readings.motor_currents[1];
    global_debug_buffer.samples[buffer_size].readings[2] =
        adc_readings.motor_currents[2];
    global_debug_buffer.samples[buffer_size].position =
        global_motor.load(::std::memory_order_relaxed)->wrapped_encoder();
    global_debug_buffer.size.fetch_add(1);
  }
  if (buffer_size == global_debug_buffer.samples.size()) {
    GPIOC_PCOR = (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4);
    GPIOD_PCOR = (1 << 4) | (1 << 5);

    PERIPHERAL_BITBAND(GPIOC_PDDR, 1) = 1;
    PERIPHERAL_BITBAND(GPIOC_PDDR, 2) = 1;
    PERIPHERAL_BITBAND(GPIOC_PDDR, 3) = 1;
    PERIPHERAL_BITBAND(GPIOC_PDDR, 4) = 1;
    PERIPHERAL_BITBAND(GPIOD_PDDR, 4) = 1;
    PERIPHERAL_BITBAND(GPIOD_PDDR, 5) = 1;

    PORTC_PCR1 = PORT_PCR_DSE | PORT_PCR_MUX(1);
    PORTC_PCR2 = PORT_PCR_DSE | PORT_PCR_MUX(1);
    PORTC_PCR3 = PORT_PCR_DSE | PORT_PCR_MUX(1);
    PORTC_PCR4 = PORT_PCR_DSE | PORT_PCR_MUX(1);
    PORTD_PCR4 = PORT_PCR_DSE | PORT_PCR_MUX(1);
    PORTD_PCR5 = PORT_PCR_DSE | PORT_PCR_MUX(1);
    i = 0;
  }
  ++i;
#endif

}

}  // extern "C"

void ConfigurePwmFtm(BigFTM *pwm_ftm) {
  // Put them all into combine active-high mode, and all the low ones staying on
  // all the time by default.
  pwm_ftm->C0SC = FTM_CSC_ELSA;
  pwm_ftm->C0V = 0;
  pwm_ftm->C1SC = FTM_CSC_ELSA;
  pwm_ftm->C1V = 0;
  pwm_ftm->C2SC = FTM_CSC_ELSA;
  pwm_ftm->C2V = 0;
  pwm_ftm->C3SC = FTM_CSC_ELSA;
  pwm_ftm->C3V = 0;
  pwm_ftm->C4SC = FTM_CSC_ELSA;
  pwm_ftm->C4V = 0;
  pwm_ftm->C5SC = FTM_CSC_ELSA;
  pwm_ftm->C5V = 0;
  pwm_ftm->C6SC = FTM_CSC_ELSA;
  pwm_ftm->C6V = 0;
  pwm_ftm->C7SC = FTM_CSC_ELSA;
  pwm_ftm->C7V = 0;

  pwm_ftm->COMBINE = FTM_COMBINE_SYNCEN3 /* Synchronize updates usefully */ |
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
  // Safe state for all channels is low.
  pwm_ftm->POL = 0;

  // Set the deadtime.
  pwm_ftm->DEADTIME =
      FTM_DEADTIME_DTPS(0) /* Prescaler of 1 */ | FTM_DEADTIME_DTVAL(9);

  pwm_ftm->CONF =
      FTM_CONF_BDMMOD(1) /* Set everything to POLn during debug halt */;
}

// Zeros the encoder. This involves blocking for an arbitrary length of time
// with interrupts disabled.
void ZeroMotor() {
#if 0
  while (true) {
    if (PERIPHERAL_BITBAND(GPIOB_PDIR, 11)) {
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
      : [pdir_word] "l"(&PERIPHERAL_BITBAND(GPIOB_PDIR, 11)),
        [cnt] "l"(&FTM1->CNT));
  __enable_irq();
#endif
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
  NVIC_SET_SANE_PRIORITY(IRQ_FTM0, 0x3);
  NVIC_SET_SANE_PRIORITY(IRQ_UART0_STATUS, 0xE);

  // Set the LED's pin to output mode.
  PERIPHERAL_BITBAND(GPIOC_PDDR, 5) = 1;
  PORTC_PCR5 = PORT_PCR_DSE | PORT_PCR_MUX(1);

#if 0
  PERIPHERAL_BITBAND(GPIOA_PDDR, 15) = 1;
  PORTA_PCR15 = PORT_PCR_DSE | PORT_PCR_MUX(1);
#endif

  // Set up the CAN pins.
  PORTA_PCR12 = PORT_PCR_DSE | PORT_PCR_MUX(2);
  PORTB_PCR19 = PORT_PCR_DSE | PORT_PCR_MUX(2);

  DMA.CR = M_DMA_EMLM;

  PORTB_PCR16 = PORT_PCR_DSE | PORT_PCR_MUX(3);
  PORTB_PCR17 = PORT_PCR_DSE | PORT_PCR_MUX(3);
  SIM_SCGC4 |= SIM_SCGC4_UART0;

  PrintingParameters printing_parameters;
  printing_parameters.stdout_uart_module = &UART0;
  printing_parameters.stdout_uart_module_clock_frequency = F_CPU;
  printing_parameters.stdout_uart_status_interrupt = IRQ_UART0_STATUS;
  printing_parameters.dedicated_usb = true;
  const ::std::unique_ptr<PrintingImplementation> printing =
      CreatePrinting(printing_parameters);
  printing->Initialize();

  AdcInitFet12();
  MathInit();
  delay(100);
  can_init(0, 1);

  MotorControlsImplementation controls;

  delay(100);

  // Index pin
  PORTB_PCR11 = PORT_PCR_MUX(1);
  // FTM1_QD_PH{A,B}
  PORTB_PCR0 = PORT_PCR_MUX(6);
  PORTB_PCR1 = PORT_PCR_MUX(6);

  // FTM0_CH[0-5]
  PORTC_PCR1 = PORT_PCR_DSE | PORT_PCR_MUX(4);
  PORTC_PCR2 = PORT_PCR_DSE | PORT_PCR_MUX(4);
  PORTC_PCR3 = PORT_PCR_DSE | PORT_PCR_MUX(4);
  PORTC_PCR4 = PORT_PCR_DSE | PORT_PCR_MUX(4);
  PORTD_PCR4 = PORT_PCR_DSE | PORT_PCR_MUX(4);
  PORTD_PCR5 = PORT_PCR_DSE | PORT_PCR_MUX(4);

  Motor motor(FTM0, FTM1, &controls, {&FTM0->C0V, &FTM0->C2V, &FTM0->C4V});
  motor.set_encoder_offset(810);
  motor.set_deadtime_compensation(9);
  ConfigurePwmFtm(FTM0);

  // TODO(Brian): Figure out how to avoid duplicating this code to slave one FTM
  // to another.
  FTM2->CONF = FTM_CONF_GTBEEN;
  FTM2->MODE = FTM_MODE_WPDIS;
  FTM2->MODE = FTM_MODE_WPDIS | FTM_MODE_FTMEN;
  FTM2->SC = FTM_SC_CLKS(0) /* Disable counting for now */;
  FTM2->CNTIN = 0;
  FTM2->CNT = 0;
  // TODO(Brian): Don't duplicate this.
  FTM2->MOD = BUS_CLOCK_FREQUENCY / SWITCHING_FREQUENCY;
  FTM2->OUTINIT = 0;
  // All of the channels are active high.
  FTM2->POL = 0;
  FTM2->SYNCONF = FTM_SYNCONF_HWWRBUF | FTM_SYNCONF_SWWRBUF |
                  FTM_SYNCONF_SWRSTCNT | FTM_SYNCONF_SYNCMODE;
  // Don't want any intermediate loading points.
  FTM2->PWMLOAD = 0;

  // Need to set them to some kind of output mode so we can actually change
  // them.
  FTM2->C0SC = FTM_CSC_MSA;
  FTM2->C1SC = FTM_CSC_MSA;

  // This has to happen after messing with SYNCONF, and should happen after
  // messing with various other things so the values can get flushed out of the
  // buffers.
  FTM2->SYNC =
      FTM_SYNC_SWSYNC /* Flush everything out right now */ |
      FTM_SYNC_CNTMAX /* Load new values at the end of the cycle */;
  // Wait for the software synchronization to finish.
  while (FTM2->SYNC & FTM_SYNC_SWSYNC) {
  }
  FTM2->SC = FTM_SC_CLKS(1) /* Use the system clock */ |
      FTM_SC_PS(0) /* Don't prescale the clock */;
  // TODO:
  //FTM2->MODE &= ~FTM_MODE_WPDIS;

  FTM2->EXTTRIG = FTM_EXTTRIG_CH0TRIG | FTM_EXTTRIG_CH1TRIG;

  // TODO(Brian): Don't duplicate the timer's MOD value.
  teensy::AdcDmaSampler adc_dma{BUS_CLOCK_FREQUENCY / SWITCHING_FREQUENCY};
  // ADC0_Dx0 is 1-0
  // ADC0_Dx2 is 1-2
  // ADC0_Dx3 is 2-0
  // ADC1_Dx0 is 2-0
  // ADC1_Dx3 is 1-0
  // Sample 0: 1-2,2-0
  // Sample 1: 1-2,1-0
  // Sample 2: 1-0,2-0
  // Sample 3: 23(SENSE0),18(VIN)
  adc_dma.set_adc0_samples({V_ADC_ADCH(2) | M_ADC_DIFF,
                            V_ADC_ADCH(2) | M_ADC_DIFF,
                            V_ADC_ADCH(0) | M_ADC_DIFF, V_ADC_ADCH(23)});
  adc_dma.set_adc1_samples({V_ADC_ADCH(0) | M_ADC_DIFF,
                            V_ADC_ADCH(3) | M_ADC_DIFF,
                            V_ADC_ADCH(0) | M_ADC_DIFF, V_ADC_ADCH(18)});
  adc_dma.set_ftm_delays({&FTM2->C0V, &FTM2->C1V});
  adc_dma.set_pdb_input(PDB_IN_FTM2);

  adc_dma.Initialize();
  FTM0->CONF = FTM_CONF_GTBEEN;
  motor.Init();
  global_motor.store(&motor, ::std::memory_order_relaxed);
  global_adc_dma.store(&adc_dma, ::std::memory_order_relaxed);

  // Output triggers to things like the PDBs on initialization.
  FTM0_EXTTRIG = FTM_EXTTRIG_INITTRIGEN;
  // Don't let any memory accesses sneak past here, because we actually
  // need everything to be starting up.
  __asm__("" :: : "memory");

  // Give everything a chance to get going.
  delay(100);

  printf("Ram start:   %p\n", __bss_ram_start__);
  printf("Heap start:  %p\n", __heap_start__);
  printf("Heap end:    %p\n", __brkval);
  printf("Stack start: %p\n", __stack_end__);

  printf("Going silent to zero motors...\n");
  // Give the print a chance to make it out.
  delay(100);
  ZeroMotor();

  motor.set_encoder_multiplier(-1);
  motor.set_encoder_calibration_offset(
      364 /*from running constant phases*/ - 26 /*average offset from lstsq*/ -
      14 /* compensation for going backwards */);

  printf("Zeroed motor!\n");
  // Give stuff a chance to recover from interrupts-disabled.
  delay(100);
  adc_dma.Reset();
  motor.Start();
  // Now poke the GTB to actually start both timers.
  FTM0->CONF = FTM_CONF_GTBEEN | FTM_CONF_GTBEOUT;

  NVIC_ENABLE_IRQ(IRQ_FTM0);
  GPIOC_PSOR = 1 << 5;

  constexpr bool dump_full_sample = false;
  constexpr bool dump_resist_calib = false;
  constexpr bool repeat_calib = true;
  while (true) {
    if (dump_resist_calib || dump_full_sample) {
      PORTC_PCR1 = PORT_PCR_DSE | PORT_PCR_MUX(4);
      PORTC_PCR2 = PORT_PCR_DSE | PORT_PCR_MUX(4);
      PORTC_PCR3 = PORT_PCR_DSE | PORT_PCR_MUX(4);
      PORTC_PCR4 = PORT_PCR_DSE | PORT_PCR_MUX(4);
      PORTD_PCR4 = PORT_PCR_DSE | PORT_PCR_MUX(4);
      PORTD_PCR5 = PORT_PCR_DSE | PORT_PCR_MUX(4);
      motor.Reset();
    }
    global_debug_buffer.size.store(0);
    global_debug_buffer.count.store(0);
    while (global_debug_buffer.size.load(::std::memory_order_relaxed) <
           global_debug_buffer.samples.size()) {
    }
    if (dump_resist_calib) {
      // Useful prints for when calibrating resistance/inductance of motor
      for (size_t i = 0; i < global_debug_buffer.samples.size(); ++i) {
        const auto &sample = global_debug_buffer.samples[i];
#if 1
        printf("%u, %d, %d, %d, %u, %u, %u, %u\n", i,
               sample.currents[0], sample.currents[1], sample.currents[2],
               sample.commands[0], sample.commands[1], sample.commands[2],
               sample.position);
#else
        printf("%" PRIu16 ",%" PRIu16 ",%" PRIu16 ",%" PRId16 ",%" PRId16
               ",%" PRId16 "\n",
               sample.commands[0], sample.commands[1], sample.commands[2],
               sample.readings[0], sample.readings[1], sample.readings[2]);
#endif
      }
    } else if (dump_full_sample) {
      printf("Dumping data\n");
      for (size_t i = 0; i < global_debug_buffer.samples.size(); ++i) {
        const auto &sample = global_debug_buffer.samples[i];

        printf("%u, %d, %d, %d, %u, %u, %u, %u, %d, %d, %d, %d\n", i,
               sample.currents[0], sample.currents[1], sample.currents[2],
               sample.commands[0], sample.commands[1], sample.commands[2],
               sample.position, static_cast<int>(sample.est_omega),
               sample.commanded_currents[0], sample.commanded_currents[1],
               sample.commanded_currents[2]);
      }
      printf("Done dumping data\n");
    } else {
      //const auto &sample = global_debug_buffer.samples.back();
      const DebugBuffer::Sample sample = global_debug_buffer.samples[0];
#if 1
      printf("%" PRIu32
             ", %d, %d, %d, %u, %u, %u, %u, %d, %d, %d, %d, %d, %d, %d\n",
             sample.cycles_since_start, sample.currents[0], sample.currents[1],
             sample.currents[2], sample.commands[0], sample.commands[1],
             sample.commands[2], sample.position,
             static_cast<int>(sample.est_omega), sample.commanded_currents[0],
             sample.commanded_currents[1], sample.commanded_currents[2],
             sample.total_command, static_cast<int>(sample.driver_request),
             static_cast<int>(sample.fuse_badness));
#else
      printf("%d, %d\n", static_cast<int>(sample.fuse_voltage),
             sample.fuse_current);
#endif
    }
    if (!repeat_calib) {
      while (true) {
      }
    }
  }

  return 0;
}

}  // namespace motors
}  // namespace frc971
