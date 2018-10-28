#include "motors/core/kinetis.h"

#include <inttypes.h>
#include <stdio.h>

#include <atomic>

#include "motors/core/time.h"
#include "motors/fet12/motor_controls.h"
#include "motors/motor.h"
#include "motors/peripheral/adc.h"
#include "motors/peripheral/can.h"
#include "motors/print/print.h"
#include "motors/util.h"

namespace frc971 {
namespace motors {
namespace {

struct Fet12AdcReadings {
  uint16_t motor_currents[3];
};

void AdcInitFet12() {
  AdcInitCommon();

  // M_CH0V ADC1_SE13
  PORTB_PCR7 = PORT_PCR_MUX(0);

  // M_CH1V ADC1_SE14
  PORTB_PCR10 = PORT_PCR_MUX(0);

  // M_CH2V ADC1_SE4b
  PORTC_PCR10 = PORT_PCR_MUX(0);

  // M_CH0F ADC0_SE16
  // dedicated

  // M_CH1F ADC0_SE18
  PORTE_PCR24 = PORT_PCR_MUX(0);

  // M_CH2F ADC0_SE23
  // dedicated
}

Fet12AdcReadings AdcReadFet12(const DisableInterrupts &) {
  Fet12AdcReadings r;

  ADC0_SC1A = 16;
  while (!(ADC0_SC1A & ADC_SC1_COCO)) {
  }
  ADC0_SC1A = 18;
  r.motor_currents[0] = ADC0_RA;
  while (!(ADC0_SC1A & ADC_SC1_COCO)) {
  }
  ADC0_SC1A = 23;
  r.motor_currents[1] = ADC0_RA;
  while (!(ADC0_SC1A & ADC_SC1_COCO)) {
  }
  r.motor_currents[2] = ADC0_RA;

  return r;
}

::std::atomic<Motor *> global_motor{nullptr};

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

void ftm0_isr(void) {
  Fet12AdcReadings adc_readings;
  {
    DisableInterrupts disable_interrupts;
    adc_readings = AdcReadFet12(disable_interrupts);
  }
#if 1
  printf("%" PRIu16 " %" PRIu16 " %" PRIu16 "\n",
         adc_readings.motor_currents[0], adc_readings.motor_currents[1],
         adc_readings.motor_currents[2]);
#endif
  const BalancedReadings balanced =
      BalanceSimpleReadings(adc_readings.motor_currents);

#if 0
  global_motor.load(::std::memory_order_relaxed)->HandleInterrupt(
      balanced,
      global_motor.load(::std::memory_order_relaxed)->wrapped_encoder());
#else
  (void)balanced;
#endif
  FTM0->SC &= ~FTM_SC_TOF;
  FTM0->C0V = 0;
  FTM0->C1V = 0;
  FTM0->C2V = 0;
  FTM0->C3V = 0;
  FTM0->C4V = 0;
  FTM0->C5V = 80;
  FTM0->PWMLOAD = FTM_PWMLOAD_LDOK;
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

  // Set the deadtime.
  pwm_ftm->DEADTIME =
      FTM_DEADTIME_DTPS(0) /* Prescaler of 1 */ | FTM_DEADTIME_DTVAL(9);
}

// Zeros the encoder. This involves blocking for an arbitrary length of time
// with interrupts disabled.
void ZeroMotor() {
#if 0
  while (true) {
    if (PERIPHERAL_BITBAND(GPIOA_PDIR, 7)) {
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
      : [pdir_word] "l"(&PERIPHERAL_BITBAND(GPIOA_PDIR, 7)),
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
  NVIC_SET_SANE_PRIORITY(IRQ_USBOTG, 0x7);
  NVIC_SET_SANE_PRIORITY(IRQ_FTM0, 0x3);

  // Set the LED's pin to output mode.
  PERIPHERAL_BITBAND(GPIOC_PDDR, 5) = 1;
  PORTC_PCR5 = PORT_PCR_DSE | PORT_PCR_MUX(1);

#if 0
  PERIPHERAL_BITBAND(GPIOA_PDDR, 15) = 1;
  PORTA_PCR15 = PORT_PCR_DSE | PORT_PCR_MUX(1);
#endif

  // Set up the CAN pins.
  PORTA_PCR12 = PORT_PCR_DSE | PORT_PCR_MUX(2);
  PORTA_PCR13 = PORT_PCR_DSE | PORT_PCR_MUX(2);

  DMA.CR = M_DMA_EMLM;

  PrintingParameters printing_parameters;
  printing_parameters.dedicated_usb = true;
  const ::std::unique_ptr<PrintingImplementation> printing =
      CreatePrinting(printing_parameters);
  printing->Initialize();

  AdcInitFet12();
  MathInit();
  delay(1000);
  can_init(0, 1);

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

  MotorControlsImplementation controls;

  delay(1000);

  // Index pin
  PORTA_PCR7 = PORT_PCR_MUX(1);
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
  delay(1000);
#if 0
  ZeroMotor();
#else
  (void)ZeroMotor;
#endif

  printf("Zeroed motor!\n");
  // Give stuff a chance to recover from interrupts-disabled.
  delay(100);
  motor.Start();
  NVIC_ENABLE_IRQ(IRQ_FTM0);
  GPIOC_PSOR = 1 << 5;

  float current_command = 0;
  while (true) {
    unsigned char command_data[8];
    int command_length;
    can_receive(command_data, &command_length, 0);
    if (command_length == 4) {
      uint32_t result = command_data[0] << 24 | command_data[1] << 16 |
                        command_data[2] << 8 | command_data[3];
      float current = static_cast<float>(result) / 1000.0f;

      static bool high_gear = false;
      if (controls.estimated_velocity() < -2015) {
        high_gear = true;
      }
      if (current < 1) {
        high_gear = false;
      }
      if (!high_gear) {
        current = current_command * -120.0f / 120.0f;
      } else {
        current = current_command * 115.0f / 120.0f;
      }
      motor.SetGoalCurrent(current);
      current_command = current;
    }
  }

  return 0;
}

}  // namespace motors
}  // namespace frc971
