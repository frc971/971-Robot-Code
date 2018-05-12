#include "motors/core/kinetis.h"

#include <stdio.h>

#include <atomic>

#include "motors/big/motor_controls.h"
#include "motors/core/time.h"
#include "motors/motor.h"
#include "motors/peripheral/adc.h"
#include "motors/peripheral/can.h"
#include "motors/usb/usb_serial.h"
#include "motors/util.h"

namespace frc971 {
namespace motors {
namespace {

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

extern void usb_init();
int _write(int file, char *ptr, int len) {
  (void)file;
  return usb_serial_write(0, ptr, len);
}

void __stack_chk_fail(void);

extern char *__brkval;
extern uint32_t __bss_ram_start__[];
extern uint32_t __heap_start__[];
extern uint32_t __stack_end__[];

void ftm0_isr(void) {
  MediumAdcReadings adc_readings;
  {
    DisableInterrupts disable_interrupts;
    adc_readings = AdcReadMedium(disable_interrupts);
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

  global_motor.load(::std::memory_order_relaxed)->HandleInterrupt(
      balanced,
      global_motor.load(::std::memory_order_relaxed)->wrapped_encoder());
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
    if (PERIPHERAL_BITBAND(GPIOE_PDIR, 24)) {
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
      : [pdir_word] "l"(&PERIPHERAL_BITBAND(GPIOE_PDIR, 24)),
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

  PERIPHERAL_BITBAND(GPIOA_PDDR, 15) = 1;
  PORTA_PCR15 = PORT_PCR_DSE | PORT_PCR_MUX(1);

  // Set up the CAN pins.
  PORTB_PCR18 = PORT_PCR_DSE | PORT_PCR_MUX(2);
  PORTB_PCR19 = PORT_PCR_DSE | PORT_PCR_MUX(2);

  DMA_CR = DMA_CR_EMLM;
  usb_serial_init();
  usb_descriptor_set_product_id(0x0490);
  usb_init();
  AdcInitMedium();
  MathInit();
  delay(1000);
  can_init(0, 1);

  GPIOD_PCOR = 1 << 3;
  PERIPHERAL_BITBAND(GPIOD_PDDR, 3) = 1;
  PORTD_PCR3 = PORT_PCR_DSE | PORT_PCR_MUX(1);
  delay(1000);
  GPIOD_PSOR = 1 << 3;
  delay(1000);
  GPIOD_PCOR = 1 << 3;
  delay(1000);

  MotorControlsImplementation controls;

  delay(1000);

  // Index pin
  PORTE_PCR24 = PORT_PCR_MUX(1);
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
  ZeroMotor();

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
