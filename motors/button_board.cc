// This file has the main for the Teensy on the button board.

#include <inttypes.h>
#include <stdio.h>
#include <atomic>
#include <cmath>

#include "motors/core/kinetis.h"
#include "motors/core/time.h"
#include "motors/peripheral/adc.h"
#include "motors/peripheral/can.h"
#include "motors/print/print.h"
#include "motors/usb/cdc.h"
#include "motors/usb/hid.h"
#include "motors/usb/usb.h"
#include "motors/util.h"

namespace frc971 {
namespace motors {
namespace {

struct JoystickAdcReadings {
  uint16_t analog0, analog1, analog2, analog3;
};

void AdcInitJoystick() {
  AdcInitCommon();

  // ANALOG0 ADC0_SE5b
  PORTD_PCR1 = PORT_PCR_MUX(0);
  // ANALOG1 ADC0_SE14
  PORTC_PCR0 = PORT_PCR_MUX(0);
  // ANALOG2 ADC0_SE13
  PORTB_PCR3 = PORT_PCR_MUX(0);
  // ANALOG3 ADC0_SE12
  PORTB_PCR2 = PORT_PCR_MUX(0);
}

JoystickAdcReadings AdcReadJoystick(const DisableInterrupts &) {
  JoystickAdcReadings r;

  ADC0_SC1A = 5;
  while (!(ADC0_SC1A & ADC_SC1_COCO)) {
  }
  ADC0_SC1A = 14;
  r.analog0 = ADC0_RA;
  while (!(ADC0_SC1A & ADC_SC1_COCO)) {
  }
  ADC0_SC1A = 13;
  r.analog1 = ADC0_RA;
  while (!(ADC0_SC1A & ADC_SC1_COCO)) {
  }
  ADC0_SC1A = 12;
  r.analog2 = ADC0_RA;
  while (!(ADC0_SC1A & ADC_SC1_COCO)) {
  }
  r.analog3 = ADC0_RA;

  return r;
}

// The HID report descriptor we use.
constexpr char kReportDescriptor1[] = {
    0x05, 0x01,        // Usage Page (Generic Desktop),
    0x09, 0x04,        // Usage (Joystick),
    0xA1, 0x01,        // Collection (Application),
    0x75, 0x08,        //     Report Size (8),
    0x95, 0x04,        //     Report Count (4),
    0x15, 0x00,        //     Logical Minimum (0),
    0x26, 0xFF, 0x00,  //     Logical Maximum (255),
    0x35, 0x00,        //     Physical Minimum (0),
    0x46, 0xFF, 0x00,  //     Physical Maximum (255),
    0x09, 0x30,        //     Usage (X),
    0x09, 0x31,        //     Usage (Y),
    0x09, 0x32,        //     Usage (Z),
    0x09, 0x33,        //     Usage (Rz),
    0x81, 0x02,        //     Input (Variable),
    0x75, 0x01,        //     Report Size (1),
    0x95, 0x10,        //     Report Count (16),
    0x25, 0x01,        //     Logical Maximum (1),
    0x45, 0x01,        //     Physical Maximum (1),
    0x05, 0x09,        //     Usage Page (Button),
    0x19, 0x01,        //     Usage Minimum (01),
    0x29, 0x10,        //     Usage Maximum (16),
    0x81, 0x02,        //     Input (Variable),
    0xC0               // End Collection
};

constexpr uint16_t report_size() { return 1 * 4 + 2; }

char DecodeAnalog(int analog) {
  // None: 132
  // Far: 71
  // Near: 103
  // Both: 0
  if (analog < 30) {
    return 0x3;
  } else if (::std::abs(analog - 71) < 10) {
    return 0x2;
  } else if (::std::abs(analog - 103) < 10) {
    return 0x1;
  } else {
    return 0x0;
  }
}

void SendJoystickData(teensy::HidFunction *joystick0,
                      teensy::HidFunction *joystick1) {
  uint32_t start = micros();
  while (true) {
    JoystickAdcReadings adc;
    char report0[report_size()];
    char report1[report_size()];
    {
      DisableInterrupts disable_interrupts;
      adc = AdcReadJoystick(disable_interrupts);
    }

    FTM0->C1V = adc.analog0 / 4;
    FTM0->C0V = adc.analog1 / 4;
    FTM0->C4V = adc.analog2 / 4;
    FTM0->C3V = adc.analog3 / 4;
    FTM0->PWMLOAD = FTM_PWMLOAD_LDOK;
    report0[0] = report1[0] = adc.analog0 / 16;
    report0[1] = report1[1] = adc.analog1 / 16;
    report0[2] = report1[2] = adc.analog2 / 16;
    report0[3] = report1[3] = adc.analog3 / 16;

    report0[4] = ((PERIPHERAL_BITBAND(GPIOD_PDIR, 5) << 0) |
                  (PERIPHERAL_BITBAND(GPIOD_PDIR, 6) << 1) |
                  (PERIPHERAL_BITBAND(GPIOB_PDIR, 0) << 2) |
                  (PERIPHERAL_BITBAND(GPIOB_PDIR, 1) << 3) |
                  (PERIPHERAL_BITBAND(GPIOA_PDIR, 14) << 4) |
                  (PERIPHERAL_BITBAND(GPIOE_PDIR, 26) << 5) |
                  (PERIPHERAL_BITBAND(GPIOA_PDIR, 16) << 6) |
                  (PERIPHERAL_BITBAND(GPIOA_PDIR, 15) << 7)) ^
                 0xff;

    report0[5] = ((PERIPHERAL_BITBAND(GPIOE_PDIR, 25) << 0) |
                  (PERIPHERAL_BITBAND(GPIOE_PDIR, 24) << 1) |
                  (PERIPHERAL_BITBAND(GPIOC_PDIR, 3) << 2) |
                  (PERIPHERAL_BITBAND(GPIOC_PDIR, 7) << 3) |
                  (PERIPHERAL_BITBAND(GPIOD_PDIR, 3) << 4) |
                  (PERIPHERAL_BITBAND(GPIOD_PDIR, 2) << 5) |
                  (PERIPHERAL_BITBAND(GPIOD_PDIR, 7) << 6) |
                  (PERIPHERAL_BITBAND(GPIOA_PDIR, 13) << 7)) ^
                 0xff;

    report1[4] =
        ((PERIPHERAL_BITBAND(GPIOA_PDIR, 12) << 0) |
         (PERIPHERAL_BITBAND(GPIOD_PDIR, 0) << 1) |
         (PERIPHERAL_BITBAND(GPIOB_PDIR, 17) << 2) |
         (PERIPHERAL_BITBAND(GPIOB_PDIR, 16) << 3) | (DecodeAnalog(report1[0]) << 4) |
         (DecodeAnalog(report1[1]) << 6)) ^
        0x0f;
    report1[5] = (DecodeAnalog(report1[2])) | (DecodeAnalog(report1[3]) << 2);

    {
      DisableInterrupts disable_interrupts;
      joystick0->UpdateReport(report0, sizeof(report0), disable_interrupts);
      joystick1->UpdateReport(report1, sizeof(report1), disable_interrupts);
    }

    start = delay_from(start, 1);
  }
}

void SetupLedFtm(BigFTM *ftm) {
  // PWMSYNC doesn't matter because we set SYNCMODE down below.
  ftm->MODE = FTM_MODE_WPDIS;
  ftm->MODE = FTM_MODE_WPDIS | FTM_MODE_FTMEN;
  ftm->SC = FTM_SC_CLKS(0) /* Disable counting for now */;

  // Use center-aligned high-true for all the channels.
  ftm->C0SC = FTM_CSC_ELSB;
  ftm->C0V = 0;
  ftm->C1SC = FTM_CSC_ELSB;
  ftm->C1V = 0;
  ftm->C2SC = FTM_CSC_ELSB;
  ftm->C2V = 0;
  ftm->C3SC = FTM_CSC_ELSB;
  ftm->C3V = 0;
  ftm->C4SC = FTM_CSC_ELSB;
  ftm->C4V = 0;
  ftm->C5SC = FTM_CSC_ELSB;
  ftm->C5V = 0;
  ftm->C6SC = FTM_CSC_ELSB;
  ftm->C6V = 0;
  ftm->C7SC = FTM_CSC_ELSB;
  ftm->C7V = 0;

  ftm->COMBINE = FTM_COMBINE_SYNCEN3 /* Synchronize updates usefully */ |
                 FTM_COMBINE_SYNCEN2 /* Synchronize updates usefully */ |
                 FTM_COMBINE_SYNCEN1 /* Synchronize updates usefully */ |
                 FTM_COMBINE_SYNCEN0 /* Synchronize updates usefully */;

  ftm->CNTIN = 0;
  ftm->CNT = 0;
  ftm->MOD = 1024;
  ftm->OUTINIT = 0;
  ftm->POL = 0;
  ftm->SYNCONF =
      FTM_SYNCONF_HWWRBUF /* Hardware trigger flushes switching points */ |
      FTM_SYNCONF_SWWRBUF /* Software trigger flushes switching points */ |
      FTM_SYNCONF_SWRSTCNT /* Software trigger resets the count */ |
      FTM_SYNCONF_SYNCMODE /* Use the new synchronization mode */;
  // Don't want any intermediate loading points.
  ftm->PWMLOAD = 0;

  ftm->SYNC = FTM_SYNC_SWSYNC /* Flush everything out right now */;
  // Wait for the software synchronization to finish.
  while (ftm->SYNC & FTM_SYNC_SWSYNC) {
  }
  ftm->SC = FTM_SC_CPWMS /* Center-aligned PWM */ |
            FTM_SC_CLKS(1) /* Use the system clock */ |
            FTM_SC_PS(6) /* Prescaler=64 */;

  ftm->MODE &= ~FTM_MODE_WPDIS;
}

}  // namespace

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

}  // extern "C"

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

  // Set all the LED pins to output, slew rate controlled, high drive strength.
  // Builtin
  PERIPHERAL_BITBAND(GPIOC_PDOR, 5) = 1;
  PORTC_PCR5 = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(1);
  PERIPHERAL_BITBAND(GPIOC_PDDR, 5) = 1;
  // LED0 FTM0_CH1
  PERIPHERAL_BITBAND(GPIOC_PDOR, 2) = 0;
  PORTC_PCR2 = PORT_PCR_DSE | PORT_PCR_ODE | PORT_PCR_SRE | PORT_PCR_MUX(4);
  PERIPHERAL_BITBAND(GPIOC_PDDR, 2) = 1;
  // LED1 FTM0_CH0
  PERIPHERAL_BITBAND(GPIOC_PDOR, 1) = 0;
  PORTC_PCR1 = PORT_PCR_DSE | PORT_PCR_ODE | PORT_PCR_SRE | PORT_PCR_MUX(4);
  PERIPHERAL_BITBAND(GPIOC_PDDR, 1) = 1;
  // LED2 FTM0_CH4
  PERIPHERAL_BITBAND(GPIOD_PDOR, 4) = 0;
  PORTD_PCR4 = PORT_PCR_DSE | PORT_PCR_ODE | PORT_PCR_SRE | PORT_PCR_MUX(4);
  PERIPHERAL_BITBAND(GPIOD_PDDR, 4) = 1;
  // LED3 FTM0_CH3
  PERIPHERAL_BITBAND(GPIOC_PDOR, 4) = 0;
  PORTC_PCR4 = PORT_PCR_DSE | PORT_PCR_ODE | PORT_PCR_SRE | PORT_PCR_MUX(4);
  PERIPHERAL_BITBAND(GPIOC_PDDR, 4) = 1;
  // LED4 FTM3_CH4 yellow
  PERIPHERAL_BITBAND(GPIOC_PDOR, 8) = 0;
  PORTC_PCR8 = PORT_PCR_DSE | PORT_PCR_ODE | PORT_PCR_SRE | PORT_PCR_MUX(1);
  PERIPHERAL_BITBAND(GPIOC_PDDR, 8) = 1;
  // LED5 FTM3_CH5 green
  PERIPHERAL_BITBAND(GPIOC_PDOR, 9) = 0;
  PORTC_PCR9 = PORT_PCR_DSE | PORT_PCR_ODE | PORT_PCR_SRE | PORT_PCR_MUX(1);
  PERIPHERAL_BITBAND(GPIOC_PDDR, 9) = 1;
  // LED6 FTM3_CH6 red
  PERIPHERAL_BITBAND(GPIOC_PDOR, 10) = 0;
  PORTC_PCR10 = PORT_PCR_DSE | PORT_PCR_ODE | PORT_PCR_SRE | PORT_PCR_MUX(1);
  PERIPHERAL_BITBAND(GPIOC_PDDR, 10) = 1;

  // Set up the CAN pins.
  PORTB_PCR18 = PORT_PCR_DSE | PORT_PCR_MUX(2);
  PORTB_PCR19 = PORT_PCR_DSE | PORT_PCR_MUX(2);

  // .1ms filter time.
  PORTA_DFWR = PORTB_DFWR = PORTC_DFWR = PORTD_DFWR = PORTE_DFWR = 6000;

  // Set up the buttons. The LEDs pull them up to 5V, so the Teensy needs to not
  // be set to pull up.
  // BTN0
  PORTD_PCR5 = PORT_PCR_MUX(1);
  PORTD_DFER |= 1 << 5;
  // BTN1
  PORTD_PCR6 = PORT_PCR_MUX(1);
  PORTD_DFER |= 1 << 6;
  // BTN2
  PORTB_PCR0 = PORT_PCR_MUX(1);
  PORTB_DFER |= 1 << 0;
  // BTN3
  PORTB_PCR1 = PORT_PCR_MUX(1);
  PORTB_DFER |= 1 << 1;
  // BTN4
  PORTA_PCR14 = PORT_PCR_MUX(1);
  PORTA_DFER |= 1 << 14;
  // BTN5
  PORTE_PCR26 = PORT_PCR_MUX(1);
  PORTE_DFER |= 1 << 26;
  // BTN6
  PORTA_PCR16 = PORT_PCR_MUX(1);
  PORTA_DFER |= 1 << 16;
  // BTN7
  PORTA_PCR15 = PORT_PCR_MUX(1);
  PORTA_DFER |= 1 << 15;
  // BTN8
  PORTE_PCR25 = PORT_PCR_MUX(1);
  PORTE_DFER |= 1 << 25;
  // BTN9
  PORTE_PCR24 = PORT_PCR_MUX(1);
  PORTE_DFER |= 1 << 24;
  // BTN10
  PORTC_PCR3 = PORT_PCR_MUX(1);
  PORTC_DFER |= 1 << 3;
  // BTN11
  PORTC_PCR7 = PORT_PCR_MUX(1);
  PORTC_DFER |= 1 << 7;
  // BTN12
  PORTD_PCR3 = PORT_PCR_MUX(1);
  PORTD_DFER |= 1 << 3;
  // BTN13
  PORTD_PCR2 = PORT_PCR_MUX(1);
  PORTD_DFER |= 1 << 2;
  // BTN14
  PORTD_PCR7 = PORT_PCR_MUX(1);
  PORTD_DFER |= 1 << 7;
  // BTN15
  PORTA_PCR13 = PORT_PCR_MUX(1);
  PORTA_DFER |= 1 << 13;
  // BTN16
  PORTA_PCR12 = PORT_PCR_MUX(1);
  PORTA_DFER |= 1 << 12;
  // BTN17
  PORTD_PCR0 = PORT_PCR_MUX(1);
  PORTD_DFER |= 1 << 0;
  // BTN18
  PORTB_PCR17 = PORT_PCR_MUX(1);
  PORTB_DFER |= 1 << 17;
  // BTN19
  PORTB_PCR16 = PORT_PCR_MUX(1);
  PORTB_DFER |= 1 << 16;

  delay(100);

  teensy::UsbDevice usb_device(0, 0x16c0, 0x0492);
  usb_device.SetManufacturer("FRC 971 Spartan Robotics");
  usb_device.SetProduct("Spartan Joystick Board");

  teensy::HidFunction joystick0(&usb_device, report_size());
  joystick0.set_report_descriptor(
      ::std::string(kReportDescriptor1, sizeof(kReportDescriptor1)));

  teensy::HidFunction joystick1(&usb_device, report_size());
  joystick1.set_report_descriptor(
      ::std::string(kReportDescriptor1, sizeof(kReportDescriptor1)));

  teensy::AcmTty tty1(&usb_device);
  PrintingParameters printing_parameters;
  printing_parameters.stdout_tty = &tty1;

  const ::std::unique_ptr<PrintingImplementation> printing =
      CreatePrinting(printing_parameters);
  usb_device.Initialize();
  printing->Initialize();

  can_init(0, 1);
  AdcInitJoystick();
  SetupLedFtm(FTM0);
  SetupLedFtm(FTM3);

  // Leave the LEDs on for a bit longer.
  delay(300);
  printf("Done starting up\n");

  // Done starting up, now turn all the LEDs off.
  PERIPHERAL_BITBAND(GPIOC_PDOR, 5) = 0;
  PERIPHERAL_BITBAND(GPIOC_PDOR, 2) = 1;
  PERIPHERAL_BITBAND(GPIOC_PDOR, 1) = 1;
  PERIPHERAL_BITBAND(GPIOD_PDOR, 4) = 1;
  PERIPHERAL_BITBAND(GPIOC_PDOR, 4) = 1;
  PERIPHERAL_BITBAND(GPIOC_PDOR, 8) = 1;
  PERIPHERAL_BITBAND(GPIOC_PDOR, 9) = 1;
  PERIPHERAL_BITBAND(GPIOC_PDOR, 10) = 1;

  SendJoystickData(&joystick0, &joystick1);

  return 0;
}

}  // namespace motors
}  // namespace frc971
