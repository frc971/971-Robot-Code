#include "motors/core/kinetis.h"

#include <inttypes.h>
#include <stdio.h>

#include <atomic>

#include "motors/core/time.h"
#include "motors/fet12/current_equalization.h"
#include "motors/fet12/motor_controls.h"
#include "motors/motor.h"
#include "motors/peripheral/adc.h"
#include "motors/peripheral/can.h"
#include "motors/peripheral/uart.h"
#include "motors/util.h"
#include "third_party/GSL/include/gsl/gsl"

namespace frc971 {
namespace motors {
namespace {

constexpr double Kv = 22000.0 * 2.0 * M_PI / 60.0 / 30.0 * 3.6;
constexpr double kVcc = 31.5;
constexpr double kIcc = 125.0;
constexpr double kR = 0.0084;

struct Fet12AdcReadings {
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

Fet12AdcReadings AdcReadFet12(const DisableInterrupts &) {
  Fet12AdcReadings r;

  ADC1_SC1A = 5;
  ADC0_SC1A = 23;
  while (!(ADC1_SC1A & ADC_SC1_COCO)) {
  }
  ADC1_SC1A = 6;
  r.motor_currents[0] = static_cast<int16_t>(ADC1_RA) - 2032;
  while (!(ADC0_SC1A & ADC_SC1_COCO)) {
  }
  ADC0_SC1A = 13;
  r.throttle = static_cast<int16_t>(ADC0_RA);
  while (!(ADC1_SC1A & ADC_SC1_COCO)) {
  }
  ADC1_SC1A = 7;
  r.motor_currents[1] = static_cast<int16_t>(ADC1_RA) - 2032;
  while (!(ADC0_SC1A & ADC_SC1_COCO)) {
  }
  r.fuse_voltage = static_cast<int16_t>(ADC0_RA);
  while (!(ADC1_SC1A & ADC_SC1_COCO)) {
  }
  r.motor_currents[2] = static_cast<int16_t>(ADC1_RA) - 2032;

  return r;
}

::std::atomic<Motor *> global_motor{nullptr};
::std::atomic<teensy::InterruptBufferedUart *> global_stdout{nullptr};

extern "C" {

void uart0_status_isr(void) {
  teensy::InterruptBufferedUart *const tty =
      global_stdout.load(::std::memory_order_relaxed);
  DisableInterrupts disable_interrupts;
  tty->HandleInterrupt(disable_interrupts);
}

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
  teensy::InterruptBufferedUart *const tty =
      global_stdout.load(::std::memory_order_acquire);
  if (tty != nullptr) {
    DisableInterrupts disable_interrupts;
    tty->Write(gsl::make_span(ptr, len), disable_interrupts);
    return len;
  }
  return 0;
}

void __stack_chk_fail(void);

extern char *__brkval;
extern uint32_t __bss_ram_start__[];
extern uint32_t __heap_start__[];
extern uint32_t __stack_end__[];

struct DebugBuffer {
  struct Sample {
    ::std::array<int16_t, 3> currents;
    ::std::array<int16_t, 3> commanded_currents;
    ::std::array<uint16_t, 3> commands;
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
  const auto wrapped_encoder =
      global_motor.load(::std::memory_order_relaxed)->wrapped_encoder();
  Fet12AdcReadings adc_readings;
  {
    DisableInterrupts disable_interrupts;
    adc_readings = AdcReadFet12(disable_interrupts);
  }
  const ::std::array<float, 3> decoupled =
      DecoupleCurrents(adc_readings.motor_currents);

  const BalancedReadings balanced =
      BalanceSimpleReadings(decoupled);

  static int i = 0;
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
  static float filtered_throttle = 0.0f;
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

  const float fuse_voltage = static_cast<float>(adc_readings.fuse_voltage);
  static float filtered_fuse_voltage = 0.0f;

  filtered_fuse_voltage =
      fuse_voltage + kFuseAlpha * (filtered_fuse_voltage - fuse_voltage);

  const float velocity =
      global_motor.load(::std::memory_order_relaxed)->estimated_velocity();
  const float bemf = velocity / (static_cast<float>(Kv) / 1.5f);
  const float abs_bemf = ::std::abs(bemf);
  constexpr float kPeakCurrent = 300.0f;
  constexpr float kLimitedCurrent = 75.0f;
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

  constexpr float kNegativeCurrent = 80.0f;
  float goal_current = -::std::min(
      filtered_throttle * (kPeakCurrent + kNegativeCurrent) - kNegativeCurrent,
      throttle_limit);

  if (velocity > -500) {
    if (goal_current > 0.0f) {
      goal_current = 0.0f;
    }
  }
  //float goal_current =
      //-::std::min(filtered_throttle * kPeakCurrent, throttle_limit);
  const float fuse_current =
      goal_current *
      (bemf + goal_current * static_cast<float>(kR) * 1.5f) /
      static_cast<float>(kVcc);
  const int16_t fuse_current_10 = static_cast<int16_t>(10.0f * fuse_current);
  fuse_badness += 0.00002f * (fuse_current * fuse_current - fuse_badness);

  global_motor.load(::std::memory_order_relaxed)
      ->SetGoalCurrent(goal_current);
  global_motor.load(::std::memory_order_relaxed)
      ->HandleInterrupt(balanced, wrapped_encoder);
#else
  (void)balanced;
  FTM0->SC &= ~FTM_SC_TOF;
  FTM0->C0V = 0;
  FTM0->C1V = 0;
  FTM0->C2V = 0;
  FTM0->C3V = 0;
  FTM0->C4V = 0;
  FTM0->C5V = 60;
  FTM0->PWMLOAD = FTM_PWMLOAD_LDOK;
#endif

  global_debug_buffer.count.fetch_add(1);

  const bool trigger = false;
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
  }

  ++i;
  if (i > 1000) {
    i = 0;
  }
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
  teensy::InterruptBufferedUart debug_uart(&UART0, F_CPU);
  debug_uart.Initialize(115200);
  global_stdout.store(&debug_uart, ::std::memory_order_release);
  NVIC_ENABLE_IRQ(IRQ_UART0_STATUS);

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
  motor.Init();
  global_motor.store(&motor, ::std::memory_order_relaxed);
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
      558 + 1034 + 39 /*big data bemf comp*/ - 14 /*just backwardsbackwards comp*/);

  printf("Zeroed motor!\n");
  // Give stuff a chance to recover from interrupts-disabled.
  delay(100);
  motor.Start();
  NVIC_ENABLE_IRQ(IRQ_FTM0);
  GPIOC_PSOR = 1 << 5;

  constexpr bool dump_full_sample = false;
  while (true) {
    if (dump_full_sample) {
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
    if (dump_full_sample) {
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
  }

  return 0;
}

}  // namespace motors
}  // namespace frc971
