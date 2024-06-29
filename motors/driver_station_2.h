#ifndef MOTORS_BUTTON_BOARD_H_
#define MOTORS_BUTTON_BOARD_H_

#include "motors/peripheral/adc.h"
#include "motors/peripheral/can.h"
#include "motors/usb/cdc.h"
#include "motors/usb/hid.h"
#include "motors/util.h"

#define NUM_ENCODERS 2
#define ENCODER_COUNTS_PER_REV 0x1000
#define ENCODER_MOD (ENCODER_COUNTS_PER_REV - 1)
#define ENCODER_RESET_TIME_MS 500
#define ENCODER_FILTER_EXPONENT 6  // depth of IIR filtering where val 2^n
#define MAX_FILTER_DELTA \
  200  // reset encoder filter when the input value exceeds filtered value by
       // this amount
#define MAX_DUTY_CYCLE_DELTA 500
#define BUTTON_BOARD_COUNT 3
#define PROCESSOR_ID_COUNT 3  // number of driver stations
#define PRINT_OFFSETS 0       // used to debug the zero's of the joystick
#define SCALED_VALUE_MAX 0xFFFF

namespace frc971 {
namespace motors {

struct JoystickAdcReadings {
  uint16_t analog0, analog1, analog2, analog3;
};

enum abs_Measurement_State {
  INIT_ABS,      // Absolute position of encoder on startup, which is treated as
                 // center
  START_PERIOD,  // Period reading needs to stay for to be considered stable.
                 // Rise-to-rise time of PWM
  WAIT_PERIOD_DONE,
  START_WIDTH,  // Rise-to-fall time of PWM
  WAIT_WIDTH_DONE
};

typedef struct {
  uint16_t enc0_trim;
  uint16_t enc1_trim;
} ENCODER_TRIMS_S;

typedef struct {
  abs_Measurement_State state;
  uint32_t period;
  uint32_t width;
  uint32_t dutycycle;
  uint32_t last_erronious_dutycycle;  // Tracks out-of-range measurements so
                                      // that duty cycle can be reset
  bool intialized;
} ABS_POSITION_S;

typedef struct {
  uint16_t angle;
  uint16_t offset;
  uint16_t resetTimer_ms;
  uint16_t filtered_duty_cycle;
  uint32_t angle_filter;
  // The encoder value at center.
  uint16_t enc_trim;
  // The tared min and max values of the limits.
  uint16_t enc_min;
  uint16_t enc_max;
} ENCODER_DATA_S;

// Identifies which board is used.
typedef struct {
  uint32_t board_id;  // There are three boards, one for each Teensy
  uint32_t
      processor_index;  // There are two processors, one for each driver station
  uint32_t can_id0;
  uint32_t can_id1;
} BOARD_CONFIG_S;

typedef struct {
  uint32_t buttons;
  uint16_t enc0;
  uint16_t enc1;
  uint16_t abs0;
  uint16_t abs1;
  uint16_t abs2;
  uint16_t abs3;
} MEASUREMENT_DATA_S;

class DriverStation2 {
 public:
  DriverStation2() = default;
  int Run();
  // Number of bytes per CAN packet
  static constexpr uint16_t kReportSize = 1 * 5 + 2;

 private:
  BOARD_CONFIG_S board_config;

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
                     MEASUREMENT_DATA_S *bbMeasurements, uint8_t board_id,
                     int can_1_board, int can_2_board);
  int ReadQuadrature(int encoderNum, uint16_t *encoderAngle);
  int MeasureAbsPosition(uint32_t encoder_id, ABS_POSITION_S *abs_position);
  int DetermineEncoderValues(ENCODER_DATA_S *enc, ABS_POSITION_S *absAngle,
                             int encoderNum, uint16_t resetTime_ms);
  void ZeroMeasurements(MEASUREMENT_DATA_S *bbMeasurements, uint32_t board);
  int UpdateMeasurementsFromCAN(MEASUREMENT_DATA_S *bbMeasurements,
                                uint8_t *canRX_data);
  void PackMeasurementsToCAN(MEASUREMENT_DATA_S *bbMeasurements,
                             uint8_t *canTX_data);
  uint32_t ReadButtons();

  int MeasurementsToJoystick(MEASUREMENT_DATA_S bbMeasurement);
};
}  // namespace motors
}  // namespace frc971

#endif  //  MOTORS_BUTTON_BOARD_H_