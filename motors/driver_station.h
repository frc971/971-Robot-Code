#ifndef MOTORS_BUTTON_BOARD_H_
#define MOTORS_BUTTON_BOARD_H_

#include "motors/peripheral/adc.h"
#include "motors/peripheral/can.h"
#include "motors/usb/cdc.h"
#include "motors/usb/hid.h"
#include "motors/util.h"

namespace frc971 {
namespace motors {

constexpr int kNumEncoders = 2;
constexpr uint16_t kEncoderCountsPerRev = 0x1000;
constexpr int kEncoderMod = kEncoderCountsPerRev - 1;
constexpr int kEncoderResetTime = 500;
constexpr int kEncoderFilterExponent =
    6;  // depth of IIR filtering where val 2^n
constexpr int kMaxFilterDelta =
    200;  // reset encoder filter when the input value exceeds filtered value by
          // this amount
constexpr int kMaxDutyCycleDelta = 500;
constexpr int kButtonBoardCount = 3;
constexpr int kProcessorIdCount = 3;  // number of driver stations
constexpr int kPrintOffsets = 0;  // used to debug the zero's of the joystick
constexpr uint16_t kScaledValueMax = 0xFFFF;

struct JoystickAdcReadings {
  uint16_t analog0, analog1, analog2, analog3;
};

enum AbsMeasurementState {
  kInitAbs,      // Absolute position of encoder on startup, which is treated as
                 // center
  kStartPeriod,  // Period reading needs to stay for to be considered stable.
                 // Rise-to-rise time of PWM
  kWaitPeriodDone,
  kStartWidth,  // Rise-to-fall time of PWM
  kWaitWidthDone
};

typedef struct {
  uint16_t enc0_trim;
  uint16_t enc1_trim;
} EncoderTrim;

typedef struct {
  AbsMeasurementState state;
  uint32_t period;
  uint32_t width;
  uint32_t dutycycle;
  uint32_t last_erronious_dutycycle;  // Tracks out-of-range measurements so
                                      // that duty cycle can be reset
  bool intialized;
} AbsPosition;

typedef struct {
  uint16_t angle;
  uint16_t offset;
  uint16_t reset_timer_ms;
  uint16_t filtered_duty_cycle;
  uint32_t angle_filter;
  // The encoder value at center.
  uint16_t enc_trim;
  // The tared min and max values of the limits.
  uint16_t enc_min;
  uint16_t enc_max;
} EncoderData;

// Identifies which board is used.
typedef struct {
  uint32_t board_id;  // There are three boards, one for each Teensy
  uint32_t
      processor_index;  // There are two processors, one for each driver station
  uint32_t can_id0;
  uint32_t can_id1;
  bool is_swerve;
} BoardConfig;

typedef struct {
  uint32_t buttons;
  uint16_t adc0;
  uint16_t adc1;
  uint16_t adc2;
  uint16_t adc3;
  uint16_t enc0;
  uint16_t enc1;
} MeasurementData;

class DriverStation {
 public:
  DriverStation() = default;
  int Run();
  // Number of bytes per CAN packet
  static constexpr uint16_t kReportSize = 1 * 5 + 2;

 private:
  BoardConfig board_config;

  // Contains the while loop to read inputs, pack data, and send it via CAN and
  // USB
  void SendJoystickData(teensy::HidFunction *joystick0,
                        teensy::HidFunction *joystick1,
                        teensy::HidFunction *joystick2,
                        uint32_t processor_index);
  void EnableLeds();
  void EnableCan();
  void EnableEncoders();
  void EnableQD(LittleFTM *ftm, int encoder);
  void EnableGlitchFilter();
  void EnableButtons();
  void DisableLeds();
  void AdcInitJoystick();
  JoystickAdcReadings AdcReadJoystick(const DisableInterrupts &);
  void ComposeReport(char report[][kReportSize],
                     MeasurementData *bb_measurements, uint8_t board_id,
                     int can_1_board, int can_2_board);
  int ReadQuadrature(int encoder_num, uint16_t *encoder_angle);
  int MeasureAbsPosition(uint32_t encoder_id, AbsPosition *abs_position);
  int DetermineEncoderValues(EncoderData *enc, AbsPosition *abs_angle,
                             int encoder_num, uint16_t reset_time_ms);
  void ZeroMeasurements(MeasurementData *bb_measurements, uint32_t board);
  int UpdateMeasurementsFromCAN(MeasurementData *bb_measurements,
                                uint8_t *can_rx_data);
  void PackMeasurementsToCAN(MeasurementData *bb_measurements,
                             uint8_t *can_tx_data);
  uint32_t ReadButtons();

  int MeasurementsToJoystick(MeasurementData bbMeasurement);
};
}  // namespace motors
}  // namespace frc971

#endif  //  MOTORS_BUTTON_BOARD_H_
