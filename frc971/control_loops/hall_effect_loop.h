#ifndef FRC971_CONTROL_LOOPS_HALL_EFFECT_H_
#define FRC971_CONTROL_LOOPS_HALL_EFFECT_H_

#include <memory>
#include <array>

#include "aos/common/control_loop/ControlLoop.h"
#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

// A parent class for creating things such as the Angle Adjust and Wrist
// which zero to some n number of hall effect sensors.
// kNumHallEffect is the number of hall effect sensors being used.
template <int kNumHallEffect>
class HallEffectLoop {
 public:
  // zero_down refers to whether the device should zero by traveling
  // down or up. max_zeroing_voltage is the maximum voltage to be applied
  // while zeroing.
  HallEffectLoop(StateFeedbackLoop<2, 1, 1> *state_feedback_loop,
                 bool zero_down, double max_zeroing_voltage);
  // UpdateZeros deals with all of the zeroing code and takes the hall
  // effect constants.
  // hall_effect_angle is the angle of the appropriate edge of the sensor.
  // This should match the zero_down variable. If it should zero by going
  // down it should be the upper edge, or vice-versa.
  // hall_effect and calibration are the hall effect values and calibration
  // values from the queue.
  // zeroing_speed is from the constants file and is the speed at which
  // to move the device when zeroing, for safety.
  // position is the encoder value, which should be position->before_angle
  // from the queues.
  // good_position is whether or not RunItertion had a good position value.
  // Note: Does NOT perform the Update operation the state feedback loop.
  void UpdateZeros(
      ::std::array<double, kNumHallEffect> hall_effect_angle,
      ::std::array<bool, kNumHallEffect> hall_effect,
      ::std::array<double, kNumHallEffect> calibration,
      double zeroing_speed,
      double position, bool good_position=true);

  static const double dt;

  const double kMaxZeroingVoltage;

  // Whether to zero down or up.
  bool zero_down_;

  // Enum to store the state of the internal zeroing state machine.
  // UNINITIALIZED is if the arm (or device) is not zeroed and disabled.
  // MOVING_OFF is if the arm is on the Hall Effect sensor and still in
  // the zeroing process.
  // ZEROING is if it is off the sensor and trying to find it.
  // READY is if it is zeroed and good for operation.
  // ESTOP doesn't do anything in terms of updating the goal or the such.
  enum State {
    UNINITIALIZED,
    MOVING_OFF,
    ZEROING,
    READY,
    ESTOP
  };

  // Internal state for zeroing.
  State state_;

  // Returns the number of the activated Hall Effect sensor, given
  // an array of all the hall effect sensors.
  // if no sensor is currently activated, then return -1.
  int HallEffect() const;
  // Returns which Hall Effect sensor has a calibration value between
  // last_off_position_ and the current_position. If none does,
  // then returns -1.
  int WhichHallEffect() const;

  // Limits the zeroing_position to prevent it from increasing beyond
  // where the goal is high enough to get the max voltage you want while
  // zeroing.
  void LimitZeroingGoal();

  // The state feedback control loop to talk to.
  ::std::unique_ptr<StateFeedbackLoop<2, 1, 1>> loop_;

  // Cache values for hall effect sensors.
  ::std::array<double, kNumHallEffect> hall_effect_angle_;
  ::std::array<bool, kNumHallEffect> hall_effect_;
  ::std::array<double, kNumHallEffect> calibration_;
  double zeroing_speed_;
  // Most recent encoder value and the one before it.
  double current_position_;
  double last_off_position_;
  double absolute_position_;
  // The sensor last used for calibration.
  int last_calibration_sensor_;
  // Position that is incremented for the goal when zeroing at Hall Effect.
  double zeroing_position_;
  // The offset of the encoder value from the actual position.
  double zero_offset_;
  double old_zero_offset_;
};

template <int kNumHallEffect>
/*static*/ const double HallEffectLoop<kNumHallEffect>::dt = 0.01;

}  // namespace control_loops
}  // namespace frc971
#endif // FRC971_CONTROL_LOOPS_HALL_EFFECT_H_
