// This file has the main for the Teensy on the button board.

// ButtonBoard is the main outline of how we communicate with the teensy, along
// with what we tell it to do

// ButtonBoardBB1 and ButtonBoardBB2 only differ in what teensy pins manage what
// on the pcb
// teensy (3.5) schematic: https://www.pjrc.com/teensy/schematic.html
// datasheets for the chip (MK64FX512) used in the teensy:
// https://www.pjrc.com/teensy/datasheets.html
// comments (especially on BB2) inform which pins are used

// HID and CAN packets can be found here:
// https://docs.google.com/spreadsheets/d/1JnJ0--BwiQWJ9FTQQb27fO30b_Dx7IDMHuBAAFcl3_o/edit?usp=sharing

#include "motors/driver_station.h"

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

// The HID report descriptor we use.
constexpr char kReportDescriptor1[] = {
    0x05, 0x01,        // Usage Page (Generic Desktop),
    0x09, 0x04,        // Usage (Joystick),
    0xA1, 0x01,        // Collection (Application),
    0x75, 0x08,        //     Report Size (8),
    0x95, 0x05,        //     Report Count (5),
    0x15, 0x00,        //     Logical Minimum (0),
    0x26, 0xFF, 0x00,  //     Logical Maximum (255),
    0x35, 0x00,        //     Physical Minimum (0),
    0x46, 0xFF, 0x00,  //     Physical Maximum (255),
    0x09, 0x30,        //     Usage (X),
    0x09, 0x31,        //     Usage (Y),
    0x09, 0x32,        //     Usage (Z),
    0x09, 0x33,        //     Usage (Rx),
    0x09, 0x34,        //     Usage (Ry),
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
}  // namespace

bool IsMyProcessorId(uint32_t id[4]) {
  uint32_t my_uuid[4] = {SIM_UIDH, SIM_UIDMH, SIM_UIDML, SIM_UIDL};

  for (int byte_index = 0; byte_index < 4; byte_index++) {
    if (id[byte_index] != my_uuid[byte_index]) {
      return false;
    }
  }

  return true;
}

uint8_t ProcessorIndex() {
  static uint32_t kPistolgripProcessorIds[kProcessorIdCount][4] = {
      {0x00000000, 0x00000000, 0x00000000, 0x00000000},
      {0x0036FFFF, 0xFFFFFFFF, 0x4E457285,
       0x60110022},  // One with cable tie labels
      {0x0035FFFF, 0xFFFFFFFF, 0x4E457285, 0x60130022},
  };

  for (int i = 0; i < kProcessorIdCount; i++) {
    if (IsMyProcessorId(kPistolgripProcessorIds[i])) {
      return i;
    }
  }

  return 0;
}

uint16_t TareEncoder(EncoderData *encoder_data) {
  return static_cast<uint16_t>((static_cast<uint32_t>(encoder_data->angle) -
                                static_cast<uint32_t>(encoder_data->enc_trim) +
                                0x800) &
                               0xfff);
}

// Scale and center controller readings.
//
// The encoder has a larger range of motion in the + direction than the -
// direction. We scale the readings so that min (-) maps to 0x0000, max (+) maps
// to 0xFFFF and the center is at 0x8000.
uint16_t ScaleEncoder(EncoderData *encoder_data, uint16_t adjusted_angle) {
  uint16_t result = 0x8000;
  if (adjusted_angle > 0x800) {
    const float scaled_angle =
        static_cast<float>(static_cast<int>(adjusted_angle) - 0x800) /
        static_cast<float>(encoder_data->enc_max - 0x800);
    result += std::min<int>(0xffff - 0x8000,
                            static_cast<int>(scaled_angle * (0xffff - 0x8000)));
  } else {
    const float scaled_angle =
        static_cast<float>(static_cast<int>(adjusted_angle) - 0x800) /
        static_cast<float>(0x800 - encoder_data->enc_min);
    result -= std::min<int>(0x8000, static_cast<int>(-scaled_angle * 0x8000));
  }

  return result;
}

uint16_t FilterIIR(uint32_t *filterValue, uint16_t currentValue,
                   uint16_t filterDepth, bool initFilter) {
  uint32_t localValue = 0;

  if (initFilter) {
    *filterValue = currentValue << filterDepth;
    return currentValue;
  }

  localValue = *filterValue - (*filterValue >> filterDepth);

  localValue += currentValue;

  *filterValue = localValue;

  return localValue >> filterDepth;
}

int DriverStation::DetermineEncoderValues(EncoderData *enc,
                                          AbsPosition *abs_angle,
                                          int encoder_num,
                                          uint16_t reset_time_ms) {
  uint16_t currentAngle = 0;

  if (enc->angle > kEncoderCountsPerRev) {
    enc->angle = 0;
  }

  if (ReadQuadrature(encoder_num, &currentAngle)) {
    return -1;
  }

  if (MeasureAbsPosition(encoder_num, abs_angle)) {
    return -1;
  }

  if (!abs_angle->intialized) {
    return 0;
  }

  enc->reset_timer_ms++;
  if (abs((int)(((currentAngle + enc->offset) & kEncoderMod) - enc->angle)) >
      0) {
    (void)FilterIIR(&(enc->angle_filter), abs_angle->dutycycle,
                    kEncoderFilterExponent, true);
  }

  if (abs((int)(enc->angle - abs_angle->dutycycle)) > kMaxFilterDelta &&
      enc->reset_timer_ms > 0) {
    (void)FilterIIR(&(enc->angle_filter), abs_angle->dutycycle,
                    kEncoderFilterExponent, true);
  }

  if (enc->reset_timer_ms > reset_time_ms) {
    enc->filtered_duty_cycle =
        FilterIIR(&(enc->angle_filter), abs_angle->dutycycle,
                  kEncoderFilterExponent, false);
    enc->offset =
        enc->filtered_duty_cycle -
        currentAngle; /* offset is calculated using filtered abs_position*/
    enc->offset &= kEncoderMod;
    enc->reset_timer_ms = reset_time_ms;
  }

  enc->angle = currentAngle + enc->offset;

  enc->angle &= kEncoderMod;

  return 0;
}

int DriverStation::MeasureAbsPosition(uint32_t encoder_id,
                                      AbsPosition *abs_position) {
  BigFTM *ftm = NULL;
  volatile uint32_t *volatile stat_ctrl0 = NULL;  // status and control
  volatile uint32_t *volatile stat_ctrl1 = NULL;
  volatile uint32_t *volatile channel_val0 = NULL;  // channel value
  volatile uint32_t *volatile channel_val1 = NULL;
  uint32_t decap_mask = 0;
  uint32_t decap_enable = 0;
  uint32_t channel_filter = 0;

  uint32_t initial = 0;
  uint32_t final = 0;

  switch (encoder_id) {
    case 0:
      ftm = FTM3;
      stat_ctrl0 = &(FTM3_C0SC);
      stat_ctrl1 = &(FTM3_C1SC);
      channel_val0 = &(FTM3_C0V);
      channel_val1 = &(FTM3_C1V);
      decap_mask = FTM_COMBINE_DECAP0;
      decap_enable = FTM_COMBINE_DECAPEN0;
      channel_filter = FTM_FILTER_CH0FVAL(0);

      break;
    case 1:
      ftm = FTM0;
      stat_ctrl0 = &(FTM0_C6SC);
      stat_ctrl1 = &(FTM0_C7SC);
      channel_val0 = &(FTM0_C6V);
      channel_val1 = &(FTM0_C7V);
      decap_mask = FTM_COMBINE_DECAP3;
      decap_enable = FTM_COMBINE_DECAPEN3;
      channel_filter = FTM_FILTER_CH3FVAL(0);

      break;

    default:
      return -1;
      break;
  }

  switch (abs_position->state) {
    case kInitAbs:
      ftm->MODE = FTM_MODE_WPDIS;
      ftm->MODE = FTM_MODE_WPDIS | FTM_MODE_FTMEN;

      ftm->CNTIN = 0;
      ftm->MOD = 0xFFFF;

      // Disable filters
      ftm->FILTER = channel_filter;

      ftm->COMBINE = decap_enable;
      ftm->COMBINE = decap_enable | decap_mask;

      // Use PWM decoding rising and falling edges in CH0 and rising edges only
      // in CH1.
      *stat_ctrl0 = FTM_CSC_ELSA;
      *stat_ctrl1 = FTM_CSC_ELSA;
      *channel_val0 = 0;
      *channel_val1 = 0;

      ftm->SYNCONF =
          FTM_SYNCONF_SWWRBUF /* Software trigger flushes MOD */ |
          FTM_SYNCONF_SWRSTCNT /* Software trigger resets the count */ |
          FTM_SYNCONF_SYNCMODE /* Use the new synchronization mode */;

      ftm->SYNC = FTM_SYNC_SWSYNC /* Flush everything out right now */;

      // Wait for the software synchronization to finish.
      while (ftm->SYNC & FTM_SYNC_SWSYNC) {
      }

      ftm->SC = FTM_SC_CLKS(1) /* Use the system clock */ |
                FTM_SC_PS(4) /* Prescaler=16 */;

      ftm->MODE &= ~FTM_MODE_WPDIS;
      abs_position->state = kStartPeriod;
      break;

    case kStartPeriod:
      if ((ftm->COMBINE & decap_mask) != decap_mask) {
        ftm->COMBINE = decap_enable | decap_mask;
        *stat_ctrl0 = FTM_CSC_ELSA;
        *stat_ctrl1 = FTM_CSC_ELSA;
        abs_position->state = kWaitPeriodDone;
      }
      break;

    case kWaitPeriodDone:
      initial = *channel_val0;
      final = *channel_val1;
      if ((*stat_ctrl0 & FTM_CSC_CHF) && (*stat_ctrl1 & FTM_CSC_CHF)) {
        // Add 0x10000 to correct for rollover
        abs_position->period = (final - initial) & 0xFFFF;

        *stat_ctrl0 &= ~FTM_CSC_CHF;
        *stat_ctrl1 &= ~FTM_CSC_CHF;

        abs_position->state = kStartWidth;
      }
      break;

    case kStartWidth:
      if ((ftm->COMBINE & decap_mask) != decap_mask) {
        ftm->COMBINE = decap_enable | decap_mask;
        *stat_ctrl0 = FTM_CSC_ELSA;
        *stat_ctrl1 = FTM_CSC_ELSB;
        abs_position->state = kWaitWidthDone;
      }
      break;

    case kWaitWidthDone:
      initial = *channel_val0;
      final = *channel_val1;
      if ((*stat_ctrl0 & FTM_CSC_CHF) && (*stat_ctrl1 & FTM_CSC_CHF)) {
        // Add 0x10000 to correct for rollover
        abs_position->width = (final - initial) & 0xFFFF;

        *stat_ctrl0 &= ~FTM_CSC_CHF;
        *stat_ctrl1 &= ~FTM_CSC_CHF;

        if (abs_position->period != 0) {
          if (((abs_position->width * kEncoderCountsPerRev) /
               abs_position->period) > kEncoderMod) {
            abs_position->state = kStartPeriod;
            break;
          }

          // Handle duty cycle out of range. Reset at next reasonable
          // measurement.
          if (abs_position->last_erronious_dutycycle ==
              (abs_position->width * kEncoderCountsPerRev) /
                  abs_position->period) {
            abs_position->dutycycle =
                (abs_position->width * kEncoderCountsPerRev) /
                abs_position->period;

            abs_position->state = kStartPeriod;
            break;
          }

          if (abs((int)((abs_position->width * kEncoderCountsPerRev) /
                        abs_position->period) -
                  (int)abs_position->dutycycle) > kMaxDutyCycleDelta) {
            abs_position->last_erronious_dutycycle =
                (abs_position->width * kEncoderCountsPerRev) /
                abs_position->period;

            abs_position->state = kStartPeriod;
            break;
          }

          abs_position->dutycycle =
              (abs_position->width * kEncoderCountsPerRev) /
              abs_position->period;
          abs_position->intialized = true;
        } else {
          abs_position->period = 0xFFFF;
        }

        abs_position->state = kStartPeriod;
      }
      break;

    default:
      return -1;
      break;
  }

  return 0;
}

std::array<EncoderData, 2> MakeEncoderData(uint32_t processor_index) {
  switch (processor_index) {
    case 0:
    default:
      return {EncoderData{
                  .angle = 0,
                  .offset = 0,
                  .reset_timer_ms = 0,
                  .filtered_duty_cycle = 0,
                  .angle_filter = 0,
                  .enc_trim = 0,
                  .enc_min = 0,
                  .enc_max = 0,
              },
              EncoderData{
                  .angle = 0,
                  .offset = 0,
                  .reset_timer_ms = 0,
                  .filtered_duty_cycle = 0,
                  .angle_filter = 0,
                  .enc_trim = 0,
                  .enc_min = 0,
                  .enc_max = 0,
              }};
    case 1:
      // Both enc_min and enc_max should come from the "trimmed" enc print
      // enc_min should be < 0x7FF and enc_max > 0x800
      // for encoder 0 they should be ~ +/- 0x050 and encoder 1 ~ +/- 0x700 from
      // center
      return {EncoderData{
                  .angle = 0,
                  .offset = 0,
                  .reset_timer_ms = 0,
                  .filtered_duty_cycle = 0,
                  .angle_filter = 0,
                  .enc_trim = 0x0568,
                  .enc_min = 0x0741,
                  .enc_max = 0x08EB,
              },
              EncoderData{
                  .angle = 0,
                  .offset = 0,
                  .reset_timer_ms = 0,
                  .filtered_duty_cycle = 0,
                  .angle_filter = 0,
                  .enc_trim = 0x0987,
                  .enc_min = 0x00EA,
                  .enc_max = 0x0FD5,
              }};
    case 2:
      return {EncoderData{
                  .angle = 0,
                  .offset = 0,
                  .reset_timer_ms = 0,
                  .filtered_duty_cycle = 0,
                  .angle_filter = 0,
                  .enc_trim = 0x02CD,
                  .enc_min = 0x0746,
                  .enc_max = 0x0900,
              },
              EncoderData{
                  .angle = 0,
                  .offset = 0,
                  .reset_timer_ms = 0,
                  .filtered_duty_cycle = 0,
                  .angle_filter = 0,
                  .enc_trim = 0x0589,
                  .enc_min = 0x00FF,
                  .enc_max = 0x0FC0,
              }};
  }
}

void DriverStation::SendJoystickData(teensy::HidFunction *joystick0,
                                     teensy::HidFunction *joystick1,
                                     teensy::HidFunction *joystick2,
                                     uint32_t processor_index) {
  std::array<EncoderData, kNumEncoders> encoder_data =
      MakeEncoderData(processor_index);

  static AbsPosition abs_position[kNumEncoders];
  memset(abs_position, 0, kNumEncoders * sizeof(AbsPosition));

  uint8_t can_data_out[8] = {0, 0, 0, 0, 0, 0, 0, 0};

  uint8_t can_data_in[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  static int length = 0;

  MeasurementData measurements[kButtonBoardCount];

  uint32_t start = micros();
  uint16_t can_timer_1 = 0;
  uint16_t can_timer_2 = 0;
  while (true) {
    JoystickAdcReadings adc;
    char report[3][kReportSize];
    {
      DisableInterrupts disable_interrupts;
      adc = AdcReadJoystick(disable_interrupts);
    }

    uint16_t tared_encoders[kNumEncoders];
    for (int i = 0; i < kNumEncoders; i++) {
      (void)DetermineEncoderValues(&encoder_data[i], &abs_position[i], i,
                                   kEncoderResetTime);
      tared_encoders[i] = TareEncoder(&encoder_data[i]);
    }

    measurements[board_config.board_id - 1].buttons = ReadButtons();
    measurements[board_config.board_id - 1].adc0 = adc.analog0;
    measurements[board_config.board_id - 1].adc1 = adc.analog1;
    measurements[board_config.board_id - 1].adc2 = adc.analog2;
    measurements[board_config.board_id - 1].adc3 = adc.analog3;
    measurements[board_config.board_id - 1].enc0 =
        ScaleEncoder(&encoder_data[0], tared_encoders[0]);
    measurements[board_config.board_id - 1].enc1 =
        ScaleEncoder(&encoder_data[1], tared_encoders[1]);

#if kPrintOffsets
    static int counter = 0;
    counter++;

    if (counter % 100 == 0) {
      printf(
          "Processor_ID: 0x%08lX 0x%08lX 0x%08lX 0x%08lX, ENC0 Max: 0x%03X, "
          "ENC0 Angle: 0x%03X, "
          "ENC0 Tared: 0x%03X, ENC0 Result: 0x%04X, ENC1 Max: 0x%03X, ENC1 "
          "Angle: 0x%03X, ENC1 "
          "Tared: 0x%03X, ENC1 Result: 0x%04X\n\r",
          SIM_UIDH, SIM_UIDMH, SIM_UIDML, SIM_UIDL, encoder_data[0].enc_max,
          encoder_data[0].angle, tared_encoders[0], measurements[2].enc0,
          encoder_data[1].enc_max, encoder_data[1].angle, tared_encoders[1],
          measurements[2].enc1);
    }
#endif

    can_receive(can_data_in, &length, 0);

    if (length == -1) {
      if (can_timer_1 < 2000) {
        can_timer_1++;
      }
    }

    if (can_timer_1 >= 2000) {
      ZeroMeasurements(measurements, board_config.can_id0 - 1);
    }

    if (length == 8) {
      UpdateMeasurementsFromCAN(measurements, can_data_in);
      can_timer_1 = 0;
    }

    can_receive(can_data_in, &length, 1);

    if (length == -1) {
      if (can_timer_2 < 2000) {
        can_timer_2++;
      }
    }

    if (can_timer_2 >= 2000) {
      ZeroMeasurements(measurements, board_config.can_id1 - 1);
    }

    if (length == 8) {
      UpdateMeasurementsFromCAN(measurements, can_data_in);
      can_timer_2 = 0;
    }

    static int counter = 0;
    counter++;

    if (counter % 100 == 0) {
      printf("CAN Timer 1: %d, from BB%ld + CAN Timer 2: %d, from BB%ld\n\r",
             can_timer_1, board_config.can_id0, can_timer_2,
             board_config.can_id1);
      printf("Report 1: 0x%04X + Report 2: 0x%04X + Report 3: 0x%04X\n\r",
             report[0][4], report[1][4], report[2][4]);
    }

    PackMeasurementsToCAN(&measurements[board_config.board_id - 1],
                          can_data_out);

    can_send(board_config.board_id, can_data_out, sizeof(can_data_out), 2);

    ComposeReport(report, measurements, board_config.board_id,
                  (can_timer_1 >= 2000) ? -1 : board_config.can_id0 - 1,
                  (can_timer_2 >= 2000) ? -1 : board_config.can_id1 - 1);

    {
      DisableInterrupts disable_interrupts;
      joystick0->UpdateReport(report[0], sizeof(report[0]), disable_interrupts);
      joystick1->UpdateReport(report[1], sizeof(report[1]), disable_interrupts);
      joystick2->UpdateReport(report[2], sizeof(report[2]), disable_interrupts);
    }

    start = delay_from(start, 1);
  }
}

void DriverStation::ZeroMeasurements(MeasurementData *bb_measurements,
                                     uint32_t board) {
  bb_measurements[board].buttons = 0;
  bb_measurements[board].adc0 = 0;
  bb_measurements[board].adc1 = 0;
  bb_measurements[board].adc2 = 0;
  bb_measurements[board].adc3 = 0;
  bb_measurements[board].enc0 = 0;
  bb_measurements[board].enc1 = 0;
}

// clang-format off
// CAN Packet Format (Swerve)
// 	      |                               Bit							                    |
// Byte	  7	        6	        5	        4	        3	        2	        1	        0
// 0	  BTN7	    BTN6	    BTN5	    BTN4	    BTN3	    BTN2	    BTN1	    BTN0
// 1	  BTN15	    BTN14	    BTN13	    BTN12	    BTN11	    BTN10	    BTN9	    BTN8
// 2	  ADC0:7	  ADC0:6	  ADC0:5	  ADC0:4	  BTN19	    BTN18	    BTN17	    BTN16
// 3	  ADC0:15	  ADC0:14	  ADC0:13	  ADC0:12	  ADC0:11	  ADC0:10	  ADC0:9	  ADC0:8
// 4	  ADC1:11	  ADC1:10	  ADC1:9	  ADC1:8	  ADC1:7	  ADC1:6	  ADC1:5	  ADC1:4
// 5	  ADC2:7	  ADC2:6	  ADC2:5	  ADC2:4	  ADC1:15	  ADC1:14	  ADC1:13	  ADC1:12
// 6	  ADC2:15	  ADC2:14	  ADC2:13	  ADC2:12	  ADC2:11	  ADC2:10	  ADC2:9	  ADC2:8
// 7	  ADC3:15	  ADC3:14	  ADC3:13	  ADC3:12	  ADC3:11	  ADC3:10	  ADC3:9	  ADC3:8
// 
// CAN Packet Format (Pistol Grip)
//       |                                Bit                                 |
// Byte	  7 	      6 	     5 	        4       	3       	2        1    	  0
// 0	  BTN7	    BTN6	   BTN5	      BTN4	    BTN3  	  BTN2  	  BTN1     BTN0
// 1	  BTN15	    BTN14	   BTN13  	  BTN12	    BTN11 	  BTN10	    BTN9	   BTN8
// 2	  -     	  -	        -	        -	        BTN19	    BTN18     BTN17    BTN16 
// 3	  ENC0:7  	ENC0:6  	ENC0:5  	ENC0:4  	ENC0:3    ENC0_2 	  ENC0:1   ENC0:0
// 4	  ENC1:3	  ENC1:2  	ENC1:1  	ENC1:0  	ENC0:11   ENC0:10   ENC0:9   ENC0:8 
// 5	  ENC1:11  	ENC1:10 	ENC1:9  	ENC1:8  	ENC1:7    ENC1:6  	ENC1:5   ENC1:4 
// 6	  ABS2:7	  ABS2:6	  ABS2:5	  ABS2:4	  ABS2:3    ABS2:2	  ABS2:1   ABS2:0 
// 7	  ABS3:7	  ABS3:6	  ABS3:5	  ABS3:4	  ABS3:3    ABS3:2  	ABS3:1   ABS3:0
//
// clang-format on
int DriverStation::UpdateMeasurementsFromCAN(MeasurementData *bb_measurements,
                                             uint8_t *can_rx_data) {
  int bb_id = 0;
  uint32_t is_swerve = 0;
  uint32_t buttons = 0;
  uint16_t enc0 = 0;
  uint16_t enc1 = 0;
  uint16_t adc0 = 0;
  uint16_t adc1 = 0;
  uint16_t adc2 = 0;
  uint16_t adc3 = 0;

  // Extract BB_id
  bb_id = (int)((can_rx_data[2] >> 2) & 0x03);
  bb_id--;

  if (bb_id == -1) {
    return -1;
  }

  // Extract is_swerve
  is_swerve = (uint32_t)((can_rx_data[2] >> 1) & 0x01);

  // Button common to both:

  buttons = (uint32_t)can_rx_data[0];
  buttons |= (uint32_t)can_rx_data[1] << 8;
  buttons |= ((uint32_t)can_rx_data[2] & 0x0F) << 16;

  bb_measurements[bb_id].buttons = buttons;

  if (bb_id == 2) {
    board_config.is_swerve = is_swerve;

    if (is_swerve) {
      adc0 = ((uint16_t)can_rx_data[2] & 0xF0);
      adc0 |= ((uint16_t)can_rx_data[3]) << 8;

      bb_measurements[bb_id].adc0 = adc0;

      adc1 = (uint16_t)can_rx_data[4];
      adc1 |= ((uint16_t)can_rx_data[5] & 0x0F) << 8;

      bb_measurements[bb_id].adc1 = adc1 << 4;

      adc2 = ((uint16_t)can_rx_data[5] & 0xF0);
      adc2 |= ((uint16_t)can_rx_data[6]) << 8;

      bb_measurements[bb_id].adc2 = adc2;

    } else {
      enc0 = (uint16_t)can_rx_data[3];
      enc0 |= ((uint16_t)can_rx_data[4] & 0x0F) << 8;

      bb_measurements[bb_id].enc0 = enc0 << 4;

      enc1 = ((uint16_t)can_rx_data[4] & 0xF0) >> 4;
      enc1 |= ((uint16_t)can_rx_data[5]) << 4;

      bb_measurements[bb_id].enc1 = enc1 << 4;

      adc2 = ((uint16_t)can_rx_data[6]) << 8;

      bb_measurements[bb_id].adc2 = adc2;
    }
    // ADC common to both:

    adc3 = ((uint16_t)can_rx_data[7]) << 8;

    bb_measurements[bb_id].adc3 = adc3;
  }

  return bb_id;
}

void DriverStation::PackMeasurementsToCAN(MeasurementData *bb_measurements,
                                          uint8_t *can_tx_data) {
  uint16_t encoder_measurements_0 = bb_measurements->enc0 >> 4;
  uint16_t encoder_measurements_1 = bb_measurements->enc1 >> 4;
  uint16_t adc_measurements_0 = bb_measurements->adc0 >> 4;
  uint16_t adc_measurements_1 = bb_measurements->adc1 >> 4;
  uint16_t adc_measurements_2 = bb_measurements->adc2 >> 4;
  uint16_t adc_measurements_3 = bb_measurements->adc3 >> 4;

  // Button (Common to both)
  can_tx_data[2] = (uint8_t)((bb_measurements->buttons >> 16) & 0x0F);
  can_tx_data[1] = (uint8_t)((bb_measurements->buttons >> 8) & 0xFF);
  can_tx_data[0] = (uint8_t)(bb_measurements->buttons & 0xFF);

  if (board_config.board_id == 3) {
    if (board_config.is_swerve) {  // if Swerve and BB3
      // Encoder
      can_tx_data[3] = (uint8_t)(encoder_measurements_0 & 0xFF);
      can_tx_data[4] = (uint8_t)((encoder_measurements_0 >> 8) & 0x0F);

      can_tx_data[4] |= (uint8_t)((encoder_measurements_1 & 0x0F) << 4);
      can_tx_data[5] = (uint8_t)((encoder_measurements_1 >> 4) & 0xFF);

      // ADC
      can_tx_data[6] = (uint8_t)(bb_measurements->adc2 >> 8);
      can_tx_data[7] = (uint8_t)(bb_measurements->adc3 >> 8);

    } else {  // if not Swerve and BB3
      // ADC
      can_tx_data[2] |= (uint8_t)((adc_measurements_0 & 0x0F) << 4);
      can_tx_data[3] = (uint8_t)((adc_measurements_0 >> 4) & 0xFF);

      can_tx_data[4] = (uint8_t)(adc_measurements_1 & 0xFF);
      can_tx_data[5] = (uint8_t)((adc_measurements_1 >> 8) & 0x0F);

      can_tx_data[5] |= (uint8_t)((adc_measurements_2 & 0x0F) << 4);
      can_tx_data[6] = (uint8_t)((adc_measurements_2 >> 4) & 0xFF);

      can_tx_data[7] = (uint8_t)(adc_measurements_3 >> 4);
    }
  }
}

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

// clang-format off
// HID Packet Format(Swerve)
// 		                           BB1/BB3 HID			                               BB2 HID		
// 	HID Word	    JS0	            JS1	            JS2	            JS3	            JS4	            JS5
// 0	AXIS0	      BB3:ADC0[15:8]  BB3:ADC1[15:8]	BB2:ADC0[15:8]	BB3:ADC0[15:8]	BB3:ADC1[15:8]	BB2:ADC0[15:8] (this one)
// 1	AXIS1	      BB3:ADC2[15:8]  BB3:ADC3[15:8]	BB2:ADC1[15:8]	BB3:ADC2[15:8]	BB3:ADC3[15:8]	BB2:ADC1[15:8] (this one)
// 2	AXIS2	      BB3:ADC2[7:0]	  0x7F	          BB2:ADC2[15:8]	BB3:ADC2[7:0]	  0x7F	          BB2:ADC2[15:8] (this one)
// 3	AXIS3	      BB3:ADC0[7:0]	  BB3:ADC1[7:0]	  BB2:ADC3[15:8]	BB3:ADC0[7:0]	  BB3:ADC1[7:0]	  BB2:ADC3[15:8] (this one)
// 4	AXIS4[0]	  BB1 _OK	        BB1 _OK	        BB1 _OK	        BB1 _OK	        BB1 _OK	        BB1 _OK
// 4	AXIS4[1]	  BB2 _OK	        BB2 _OK	        BB2 _OK	        BB2 _OK	        BB2 _OK	        BB2 _OK
// 4	AXIS4[2]	  BB3_OK	        BB3_OK	        BB3_OK	        BB3_OK	        BB3_OK	        BB3_OK
// 4	AXIS4[3:4]	0b01	          0b10	          0b11	          0b01	          0b10	          0b11
// 4	AXIS4[5]	  0	              0	              0	              1	              1	              1
// 4	AXIS[6:7]	  0b01	          0b01	          0b01	          0b01	          0b01	          0b01 (that one)
// 5	B0	        BB3:B0	        BB1:B0	        BB2:B0	        BB3:B0 	        BB1:B0          BB2:B0
// 5	B1	        BB3:B1	        BB1:B1	        BB2:B1	        BB3:B1 	        BB1:B1          BB2:B1
// 5	B2	        BB3:B2	        BB1:B2	        BB2:B2	        BB3:B2 	        BB1:B2          BB2:B2
// 5	B3	        BB3:B3	        BB1:B3	        BB2:B3	        BB3:B3 	        BB1:B3          BB2:B3
// 5	B4	        BB3:B4	        BB1:B4	        BB2:B4	        BB3:B4 	        BB1:B4          BB2:B4
// 5	B5	        BB3:B5	        BB1:B5	        BB2:B5	        BB3:B5 	        BB1:B5          BB2:B5
// 5	B6	        BB3:B6	        BB1:B6	        BB2:B6	        BB3:B6 	        BB1:B6          BB2:B6
// 5	B7	        BB3:B7	        BB1:B7	        BB2:B7	        BB3:B7 	        BB1:B7          BB2:B7
// 6	B8	        BB3:B8	        BB1:B8	        BB2:B8	        BB3:B8 	        BB1:B8          BB2:B8
// 6	B9	        BB3:B9	        BB1:B9	        BB2:B9	        BB3:B9 	        BB1:B9          BB2:B9
// 6	B10	        BB3:B10	        BB1:B10	        BB2:B10	        BB3:B10	        BB1:B10         BB2:B10
// 6	B11	        BB3:B11	        BB1:B11	        BB2:B11	        BB3:B11	        BB1:B11         BB2:B11
// 6	B12	        BB3:B12	        BB1:B12	        BB2:B12	        BB3:B12	        BB1:B12         BB2:B12
// 6	B[14:13]	  0b01	          0b10	          0b11	          0b01	          0b10	          0b11
// 6	B15	        0	              0	              0	              1	              1	              1
// 
// HID Packet Format(Pistol Grip)
//		                          BB1/BB3 HID			                                BB2 HID		
//	  HID Word	  JS0	            JS1	            JS2	            JS3	            JS4	            JS5
// 0	AXIS0	      BB3:ENC0[15:8]	BB3:ENC1[15:8]  BB2:ADC0        BB3:ENC0[15:8]	BB3:ENC1[15:8]  BB2:ADC0
// 1	AXIS1	      0x7F		        0x7F         	  BB2:ADC1        0x7F	          0x7F            BB2:ADC1
// 2	AXIS2	      0x7F	          0x7F	          BB2:ADC2        0x7F	          0x7F            BB2:ADC2
// 3	AXIS3	      BB3:ENC0[7:0]	  BB3:ENC1[7:0]	  BB2:ADC3        BB3:ENC0[7:0]	  BB3:ENC1[7:0]   BB2:ADC3
// 4	AXIS4[0]	  BB1 _OK	        BB1 _OK	        BB1 _OK	        BB1 _OK	        BB1 _OK	        BB1 _OK
// 4	AXIS4[1]	  BB2 _OK	        BB2 _OK	        BB2 _OK	        BB2 _OK	        BB2 _OK	        BB2 _OK
// 4	AXIS4[2]	  BB3_OK	        BB3_OK	        BB3_OK	        BB3_OK	        BB3_OK	        BB3_OK
// 4	AXIS4[3:4]  0b01	          0b10	          0b11	          0b01	          0b10	          0b11
// 4	AXIS4[5]	  0	              0	              0	              1	              1	              1
// 4	AXIS4[6:7]  0b00	          0b00	          0b00	          0b00	          0b00	          0b00
// 5	B0	        BB3:B0	        BB1:B8	        BB2:B4	        BB3:B0	        BB1:B8	        BB2:B4
// 5	B1	        BB3:B1	        BB1:B9	        BB2:B5	        BB3:B1	        BB1:B9	        BB2:B5
// 5	B2	        BB3:B2	        BB1:B10	        BB2:B6	        BB3:B2	        BB1:B10	        BB2:B6
// 5	B3	        BB3:B3	        BB1:B11	        BB2:B7	        BB3:B3	        BB1:B11	        BB2:B7
// 5	B4	        BB3:B4	        BB1:B12	        BB2:B8	        BB3:B4	        BB1:B12	        BB2:B8
// 5	B5	        BB1:B0	        BB1:B13	        BB2:B9	        BB1:B0	        BB1:B13	        BB2:B9
// 5	B6	        BB1:B1	        BB1:B14	        BB2:B10	        BB1:B1	        BB1:B14	        BB2:B10
// 5	B7	        BB1:B2	        BB1:B15	        BB2:B11	        BB1:B2	        BB1:B15	        BB2:B11
// 6	B8	        BB1:B3	        BB1:B16	        BB2:B12	        BB1:B3	        BB1:B16	        BB2:B12
// 6	B9	        BB1:B4	        BB2:B0	        BB2:B13	        BB1:B4	        BB2:B0	        BB2:B13
// 6	B10	        BB1:B5	        BB2:B1	        BB2:B14	        BB1:B5	        BB2:B1	        BB2:B14
// 6	B11	        BB1:B6	        BB2:B2	        BB2:B15	        BB1:B6	        BB2:B2	        BB2:B15
// 6	B12	        BB1:B7	        BB2:B3	        BB2:B16	        BB1:B7	        BB2:B3	        BB2:B16
// 6	B[14:13]	  0b01	          0b10	          0b11	          0b01	          0b10	          0b11
// 6	B15	        0	              0	              0	              1	              1	              1
//
// clang-format on
void DriverStation::ComposeReport(char report[][kReportSize],
                                  MeasurementData *bb_measurements,
                                  uint8_t board_id, int can_1_board,
                                  int can_2_board) {
  memset(report, 0, 3 * sizeof(*report));

  if (board_config.is_swerve) {  // if swerve
    // JS0 Report
    report[0][0] = (char)(bb_measurements[2].adc0 >> 8 & 0xFF);
    report[0][1] = (char)(bb_measurements[2].adc2 >> 8 & 0xFF);
    report[0][2] = (char)(bb_measurements[2].adc2 & 0xFF);
    report[0][3] = (char)(bb_measurements[2].adc0 & 0xFF);

    // JS1 Report
    report[1][0] = (char)(bb_measurements[2].adc1 >> 8 & 0xFF);
    report[1][1] = (char)(bb_measurements[2].adc3 >> 8 & 0xFF);
    report[1][2] = (char)((0x7F) & 0xFF);
    report[1][3] = (char)(bb_measurements[2].adc1 & 0xFF);
  } else {  // if not swerve
    // JS0 Report
    report[0][0] = (char)(bb_measurements[2].enc0 >> 8 & 0xFF);
    report[0][1] = (char)((0x7F) & 0xFF);
    report[0][2] = (char)((0x7F) & 0xFF);
    report[0][3] = (char)(bb_measurements[2].enc0 & 0xFF);

    // JS1 Report
    report[1][0] = (char)(((bb_measurements[2].enc1) >> 8) & 0xFF);
    report[1][1] = (char)((0x7F) & 0xFF);
    report[1][2] = (char)((0x7F) & 0xFF);
    report[1][3] = (char)((bb_measurements[2].enc1) & 0xFF);
  }

  // Common to all controller types:

  // JS2 Report
  report[2][0] = (char)((bb_measurements[1].adc0 >> 8) & 0xFF);
  report[2][1] = (char)((bb_measurements[1].adc1 >> 8) & 0xFF);
  report[2][2] = (char)((bb_measurements[1].adc2 >> 8) & 0xFF);
  report[2][3] = (char)((bb_measurements[1].adc3 >> 8) & 0xFF);

  // Telling if it's swerve controller
  for (int i = 0; i < 3; i++) {
    report[i][4] = (char)(board_config.is_swerve << 6);
  }

  // JS0 report
  report[0][4] |= (char)(1 << (board_id - 1));

  if (can_1_board != -1) {
    report[0][4] |= (char)(1 << (can_1_board));
  }
  if (can_2_board != -1) {
    report[0][4] |= (char)(1 << (can_2_board));
  }
  report[0][4] |= (char)(1 << 3);

  report[0][5] = (char)(bb_measurements[2].buttons & 0xFF);  // BB3 BTN[7:0]

  report[0][6] =
      (char)((bb_measurements[2].buttons >> 8) & 0x1F);  // BB3 BTN[12:8]
  report[0][6] |= (char)(1 << 5);

  // JS1 report
  report[1][4] |= (char)(1 << (board_id - 1));
  if (can_1_board != -1) {
    report[1][4] |= (char)(1 << (can_1_board));
  }
  if (can_2_board != -1) {
    report[1][4] |= (char)(1 << (can_2_board));
  }

  report[1][4] |= (char)(2 << 3);

  report[1][5] = (char)(bb_measurements[0].buttons & 0xFF);  // BB1 BTN[7:0]

  report[1][6] =
      (char)((bb_measurements[0].buttons >> 8) & 0x1F);  // BB1 BTN[12:8]
  report[1][6] |= (char)(2 << 5);

  // JS2 report
  report[2][4] |= (char)(1 << (board_id - 1));
  if (can_1_board != -1) {
    report[2][4] |= (char)(1 << (can_1_board));
  }
  if (can_2_board != -1) {
    report[2][4] |= (char)(1 << (can_2_board));
  }
  report[2][4] |= (char)(3 << 3);

  report[2][5] = (char)(bb_measurements[1].buttons & 0xFF);  // BB2 BTN[7:0]

  report[2][6] =
      (char)((bb_measurements[1].buttons >> 8) & 0x1F);  // BB2 BTN[12:8]
  report[2][6] |= (char)(3 << 5);

  // BB2 HID descriptor
  if (board_id == 2) {
    report[0][4] |= (char)(1 << 5);
    report[1][4] |= (char)(1 << 5);
    report[2][4] |= (char)(1 << 5);

    report[0][6] |= (char)(1 << 7);  // BB1 BTN[15]
    report[1][6] |= (char)(1 << 7);  // BB2 BTN[15]
    report[2][6] |= (char)(1 << 7);  // BB3 BTN[15]
  }

  /*
  for ( int i = 0; i < 3; i++ ) {
   printf("Report %d: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X    ", i,
  report[i][0], report[i][1], report[i][2], report[i][3], report[i][4],
  report[i][5]);
  }
  printf("\n\r\n\r");
  }
  */
}

uint32_t DriverStation::ReadButtons() {
  uint32_t buttons = 0;

  buttons = ((PERIPHERAL_BITBAND(GPIOD_PDIR, 5) << 0) |     // BTN0	PTD5
             (PERIPHERAL_BITBAND(GPIOC_PDIR, 0) << 1) |     // BTN1	PTC0
             (PERIPHERAL_BITBAND(GPIOB_PDIR, 3) << 2) |     // BTN2	PTB3
             (PERIPHERAL_BITBAND(GPIOB_PDIR, 2) << 3) |     // BTN3	PTB2
             (PERIPHERAL_BITBAND(GPIOA_PDIR, 14) << 4) |    // BTN4	PTA14
             (PERIPHERAL_BITBAND(GPIOE_PDIR, 26) << 5) |    // BTN5	PTE26
             (PERIPHERAL_BITBAND(GPIOA_PDIR, 16) << 6) |    // BTN6	PTA16
             (PERIPHERAL_BITBAND(GPIOA_PDIR, 15) << 7) |    // BTN7	PTA15
             (PERIPHERAL_BITBAND(GPIOE_PDIR, 25) << 8) |    // BTN8	PTE25
             (PERIPHERAL_BITBAND(GPIOA_PDIR, 5) << 9) |     // BTN9	PTA5
             (PERIPHERAL_BITBAND(GPIOC_PDIR, 3) << 10) |    // BTN10	PTC3
             (PERIPHERAL_BITBAND(GPIOC_PDIR, 7) << 11) |    // BTN11	PTC7
             (PERIPHERAL_BITBAND(GPIOD_PDIR, 3) << 12) |    // BTN12	PTD3
             (PERIPHERAL_BITBAND(GPIOD_PDIR, 2) << 13) |    // BTN13	PTD2
             (PERIPHERAL_BITBAND(GPIOD_PDIR, 7) << 14) |    // BTN14	PTD7
             (PERIPHERAL_BITBAND(GPIOB_PDIR, 10) << 15) |   // BTN15	PTB10
             (PERIPHERAL_BITBAND(GPIOB_PDIR, 11) << 16) |   // BTN16	PTB11
             (PERIPHERAL_BITBAND(GPIOD_PDIR, 4) << 17) |    // BTN17	PTD4
             (PERIPHERAL_BITBAND(GPIOB_PDIR, 17) << 18) |   // BTN18	PTB17
             (PERIPHERAL_BITBAND(GPIOB_PDIR, 16) << 19)) ^  // BTN19	PTB16
            0x000FFFFF;

  return buttons;
}

JoystickAdcReadings DriverStation::AdcReadJoystick(const DisableInterrupts &) {
  JoystickAdcReadings r;

  // ENC1_ABS_ADC	(PTE24) ADC0_SE17
  ADC0_SC1A = 17;
  while (!(ADC0_SC1A & ADC_SC1_COCO)) {
  }
  // ABS2_ADC	(PTC1) ADC0_SE15
  ADC0_SC1A = 15;
  r.analog1 = ADC0_RA << 4;
  while (!(ADC0_SC1A & ADC_SC1_COCO)) {
  }
  // ABS3_ADC	(PTC2) ADC0_SE4b
  ADC0_SC1A = 4;
  r.analog2 = ADC0_RA << 4;
  while (!(ADC0_SC1A & ADC_SC1_COCO)) {
  }
  // ENC0_ABS_ADC	(PTD1) ADC0_SE5b
  ADC0_SC1A = 5;
  r.analog3 = ADC0_RA << 4;
  while (!(ADC0_SC1A & ADC_SC1_COCO)) {
  }
  r.analog0 = ADC0_RA << 4;

  return r;
}

void DriverStation::AdcInitJoystick() {
  AdcInitCommon();
  // ENC1_ABS_ADC	(PTE24) ADC0_SE17
  PORTE_PCR24 = PORT_PCR_MUX(0);
  // ABS2_ADC	(PTC1) ADC0_SE15
  PORTC_PCR1 = PORT_PCR_MUX(0);
  // ABS3_ADC	(PTC2) ADC0_SE4b
  PORTC_PCR2 = PORT_PCR_MUX(0);
  // ENC0_ABS_ADC	(PTD1) ADC0_SE5b
  PORTD_PCR1 = PORT_PCR_MUX(0);
}

void DriverStation::EnableLeds() {
  // Set all the LED pins to output, drive strength enable (high drive since its
  // an output), slew rate enable (slow since output) LED (on board)	PTC5
  PERIPHERAL_BITBAND(GPIOC_PDOR, 5) = 1;
  PORTC_PCR5 = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(1);
  PERIPHERAL_BITBAND(GPIOC_PDDR, 5) = 1;
  // LED0R	PTC6
  PERIPHERAL_BITBAND(GPIOC_PDOR, 6) = 1;
  PORTC_PCR6 = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(1);
  PERIPHERAL_BITBAND(GPIOC_PDDR, 6) = 1;
  // LED0Y	PTA17
  PERIPHERAL_BITBAND(GPIOA_PDOR, 17) = 1;
  PORTA_PCR17 = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(1);
  PERIPHERAL_BITBAND(GPIOA_PDDR, 17) = 1;
  // LED0G	PTC11
  PERIPHERAL_BITBAND(GPIOC_PDOR, 11) = 1;
  PORTC_PCR11 = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(1);
  PERIPHERAL_BITBAND(GPIOC_PDDR, 11) = 1;
  // LED1R	PTC10
  PERIPHERAL_BITBAND(GPIOC_PDOR, 10) = 1;
  PORTC_PCR10 = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(1);
  PERIPHERAL_BITBAND(GPIOC_PDDR, 10) = 1;
  // LED1Y	PTC4
  PERIPHERAL_BITBAND(GPIOC_PDOR, 4) = 1;
  PORTC_PCR4 = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(1);
  PERIPHERAL_BITBAND(GPIOC_PDDR, 4) = 1;
  // LED1G	PTC8
  PERIPHERAL_BITBAND(GPIOC_PDOR, 8) = 1;
  PORTC_PCR8 = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(1);
  PERIPHERAL_BITBAND(GPIOC_PDDR, 8) = 1;
}

void DriverStation::EnableCan() {
  // Set up the CAN pins.
  // (PTA12) CAN0_TX
  PORTA_PCR12 = PORT_PCR_DSE | PORT_PCR_MUX(2);
  // (PTA13) CAN0_RX
  PORTA_PCR13 = PORT_PCR_DSE | PORT_PCR_MUX(2);
}

void DriverStation::EnableEncoders() {
  // Set up the encoder inputs
  // ENC0_A	(PTB18) FTM2_QD_PHA
  PORTB_PCR18 = PORT_PCR_MUX(6) /*| PORT_PCR_PE | PORT_PCR_PS*/;

  // ENC0_B	(PTB19) FTM2_QD_PHB
  PORTB_PCR19 = PORT_PCR_MUX(6) /*| PORT_PCR_PE | PORT_PCR_PS*/;

  // ENC0_ABS	(PTD0) FTM3_CH0
  PORTD_PCR0 = PORT_PCR_MUX(4) /*| PORT_PCR_PE | PORT_PCR_PS*/;

  // ENC1_A	(PTB1) FTM1_QD_PHB
  PORTB_PCR1 = PORT_PCR_MUX(6) /*| PORT_PCR_PE | PORT_PCR_PS*/;

  // ENC1_B	(PTB0) FTM1_QD_PHA
  PORTB_PCR0 = PORT_PCR_MUX(6) /*| PORT_PCR_PE | PORT_PCR_PS*/;

  // ENC1_ABS	(PTD6) FTM0_CH6
  PORTD_PCR6 = PORT_PCR_MUX(4) /*| PORT_PCR_PE | PORT_PCR_PS*/;
}

// Enable quadrature encoding.
void DriverStation::EnableQD(LittleFTM *ftm, int encoder) {
  ftm->MODE = FTM_MODE_WPDIS;
  ftm->MODE = FTM_MODE_WPDIS | FTM_MODE_FTMEN;

  ftm->CNTIN = 0;
  ftm->CNT = 0;
  ftm->MOD = kEncoderCountsPerRev;

  if (encoder == 1) {
    ftm->QDCTRL = FTM_QDCTRL_PHBPOL;
  }

  ftm->C0SC = FTM_CSC_ELSA;
  ftm->C1SC = FTM_CSC_ELSA;

  // Disable filters
  ftm->FILTER = FTM_FILTER_CH0FVAL(0) | FTM_FILTER_CH1FVAL(0) |
                FTM_FILTER_CH2FVAL(0) | FTM_FILTER_CH3FVAL(0);

  // Use Quadrature Encoding.
  ftm->QDCTRL |= FTM_QDCTRL_QUADEN;

  ftm->SYNCONF = FTM_SYNCONF_SWWRBUF /* Software trigger flushes MOD */ |
                 FTM_SYNCONF_SWRSTCNT /* Software trigger resets the count */ |
                 FTM_SYNCONF_SYNCMODE /* Use the new synchronization mode */;

  ftm->SYNC = FTM_SYNC_SWSYNC /* Flush everything out right now */;

  // Wait for the software synchronization to finish.
  while (ftm->SYNC & FTM_SYNC_SWSYNC) {
  }

  ftm->SC = FTM_SC_CLKS(1) /* Use the system clock */ |
            FTM_SC_PS(0) /* Prescaler=64 */;

  ftm->MODE &= ~FTM_MODE_WPDIS;
}

int DriverStation::ReadQuadrature(int encoder_num, uint16_t *encoder_angle) {
  switch (encoder_num) {
    case 0:
      *encoder_angle = FTM2_CNT;
      break;

    case 1:
      *encoder_angle = FTM1_CNT;
      break;

    default:
      return -1;
      break;
  }
  return 0;
}

void DriverStation::EnableGlitchFilter() {
  // Enable 1KHz filter clock
  PORTA_DFCR = 1;
  PORTB_DFCR = 1;
  PORTC_DFCR = 1;
  PORTD_DFCR = 1;
  PORTE_DFCR = 1;
  // 10ms filter time
  PORTA_DFWR = 10;
  PORTB_DFWR = 10;
  PORTC_DFWR = 10;
  PORTD_DFWR = 10;
  PORTE_DFWR = 10;
  // TODO: validate the 10ms gitch filter, seems to work
}

void DriverStation::EnableButtons() {
  // Set up the buttons. The LEDs pull them up to 5V, so the Teensy needs to not
  // be set to pull up.
  // BTN0	PTD5
  PORTD_PCR5 = PORT_PCR_MUX(1) | PORT_PCR_PFE;
  PORTD_DFER |= 1 << 5;

  // BTN1	PTC0
  PORTC_PCR0 = PORT_PCR_MUX(1) | PORT_PCR_PFE;
  PORTC_DFER |= 1 << 0;

  // BTN2	PTB3
  PORTB_PCR3 = PORT_PCR_MUX(1) | PORT_PCR_PFE;
  PORTB_DFER |= 1 << 3;

  // BTN3	PTB2
  PORTB_PCR2 = PORT_PCR_MUX(1) | PORT_PCR_PFE;
  PORTB_DFER |= 1 << 2;

  // BTN4	PTA14
  PORTA_PCR14 = PORT_PCR_MUX(1) | PORT_PCR_PFE;
  PORTA_DFER |= 1 << 14;

  // BTN5	PTE26
  PORTE_PCR26 = PORT_PCR_MUX(1) | PORT_PCR_PFE;
  PORTE_DFER |= 1 << 26;

  // BTN6	PTA16
  PORTA_PCR16 = PORT_PCR_MUX(1) | PORT_PCR_PFE;
  PORTA_DFER |= 1 << 16;

  // BTN7	PTA15
  PORTA_PCR15 = PORT_PCR_MUX(1) | PORT_PCR_PFE;
  PORTA_DFER |= 1 << 15;

  // BTN8	PTE25
  PORTE_PCR25 = PORT_PCR_MUX(1) | PORT_PCR_PFE;
  PORTE_DFER |= 1 << 25;

  // BTN9	PTA5
  PORTA_PCR5 = PORT_PCR_MUX(1) | PORT_PCR_PFE;
  PORTA_DFER |= 1 << 5;

  // BTN10	PTC3
  PORTC_PCR3 = PORT_PCR_MUX(1) | PORT_PCR_PFE;
  PORTC_DFER |= 1 << 3;

  // BTN11	PTC7
  PORTC_PCR7 = PORT_PCR_MUX(1) | PORT_PCR_PFE;
  PORTC_DFER |= 1 << 7;

  // BTN12	PTD3
  PORTD_PCR3 = PORT_PCR_MUX(1) | PORT_PCR_PFE;
  PORTD_DFER |= 1 << 3;

  // BTN13	PTD2
  PORTD_PCR2 = PORT_PCR_MUX(1) | PORT_PCR_PFE;
  PORTD_DFER |= 1 << 2;

  // BTN14	PTD7
  PORTD_PCR7 = PORT_PCR_MUX(1) | PORT_PCR_PFE;
  PORTD_DFER |= 1 << 7;

  // BTN15	PTB10
  PORTB_PCR10 = PORT_PCR_MUX(1) | PORT_PCR_PFE;
  PORTB_DFER |= 1 << 10;

  // BTN16	PTB11
  PORTB_PCR11 = PORT_PCR_MUX(1) | PORT_PCR_PFE;
  PORTB_DFER |= 1 << 11;

  // BTN17	PTD4
  PORTD_PCR4 = PORT_PCR_MUX(1) | PORT_PCR_PFE;
  PORTD_DFER |= 1 << 4;
  (PERIPHERAL_BITBAND(GPIOD_PDIR, 4) =
       1);  // BTN17	PTD4 is being forced to be an input

  // BTN18	PTB17
  PORTB_PCR17 = PORT_PCR_MUX(1) | PORT_PCR_PFE;
  PORTB_DFER |= 1 << 17;

  // BTN19	PTB16
  PORTB_PCR16 = PORT_PCR_MUX(1) | PORT_PCR_PFE;
  PORTB_DFER |= 1 << 16;
}

void DriverStation::DisableLeds() {
  // LED (on board)	PTC5
  PERIPHERAL_BITBAND(GPIOC_PDOR, 5) = 0;
  // LED0R	PTC6
  PERIPHERAL_BITBAND(GPIOC_PDOR, 6) = 0;
  // LED0Y	PTA17
  PERIPHERAL_BITBAND(GPIOA_PDOR, 17) = 0;
  // LED0G	PTC11
  PERIPHERAL_BITBAND(GPIOC_PDOR, 11) = 0;
  // LED1R	PTC10
  PERIPHERAL_BITBAND(GPIOC_PDOR, 10) = 0;
  // LED1Y	PTC4
  PERIPHERAL_BITBAND(GPIOC_PDOR, 4) = 0;
  // LED1G	PTC8
  PERIPHERAL_BITBAND(GPIOC_PDOR, 8) = 1;
}

int DriverStation::Run() {
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

  EnableLeds();

  EnableGlitchFilter();

  EnableCan();

  EnableEncoders();

  EnableButtons();

  delay(100);

  teensy::UsbDevice usb_device(0, 0x16c0, 0x0492);
  usb_device.SetManufacturer("FRC 971 Spartan Robotics");
  usb_device.SetProduct("Spartan Joystick Board");

  teensy::HidFunction joystick0(&usb_device, kReportSize);
  joystick0.set_report_descriptor(
      ::std::string(kReportDescriptor1, sizeof(kReportDescriptor1)));

  teensy::HidFunction joystick1(&usb_device, kReportSize);
  joystick1.set_report_descriptor(
      ::std::string(kReportDescriptor1, sizeof(kReportDescriptor1)));

  teensy::HidFunction joystick2(&usb_device, kReportSize);
  joystick2.set_report_descriptor(
      ::std::string(kReportDescriptor1, sizeof(kReportDescriptor1)));

  teensy::AcmTty tty1(&usb_device);
  PrintingParameters printing_parameters;
  printing_parameters.stdout_tty = &tty1;

  const ::std::unique_ptr<PrintingImplementation> printing =
      CreatePrinting(printing_parameters);
  usb_device.Initialize();
  printing->Initialize();

  AdcInitJoystick();

  EnableQD(FTM1, 1);
  EnableQD(FTM2, 0);

  // Leave the LEDs on for a bit longer.
  delay(300);
  printf("Done starting up\n");

  // Done starting up, now turn all the LEDs off.
  DisableLeds();

  board_config.board_id = 0;
  board_config.processor_index = 0;
  board_config.can_id0 = 0;
  board_config.can_id1 = 0;

  uint32_t button18 =
      PERIPHERAL_BITBAND(GPIOB_PDIR, 17) ^ 0x1;  // BTN18	PTB17
  uint32_t button19 =
      PERIPHERAL_BITBAND(GPIOB_PDIR, 16) ^ 0x1;  // BTN19	PTB16
  // TODO(sindy): Remove this once we have new button boards. Temp fix for old
  // driver station with broken pin 17
  uint32_t button17 = PERIPHERAL_BITBAND(GPIOD_PDIR, 4) ^ 0x1;  // BTN17 PTD4
  // PERIPHERAL_BITBAND(GPIOB_PDIR, 11) ^ 0x1;  // BTN16

  board_config.board_id = (button19 << 1) | button18;

  board_config.processor_index = ProcessorIndex();

  switch (board_config.board_id) {
    case 1:
      board_config.can_id0 = 2;
      board_config.can_id1 = 3;
      break;

    case 2:
      board_config.can_id0 = 1;
      board_config.can_id1 = 3;
      break;

    case 3:
      board_config.can_id0 = 2;
      board_config.can_id1 = 1;
      board_config.is_swerve = button17;
      break;

    default:
      board_config.board_id = 0;
      break;
  }

  can_init(board_config.can_id0, board_config.can_id1);

  SendJoystickData(&joystick0, &joystick1, &joystick2,
                   board_config.processor_index);

  return 0;
}

extern "C" int main(void) {
  frc971::motors::DriverStation driverStation;
  return driverStation.Run();
}

}  // namespace motors
}  // namespace frc971
