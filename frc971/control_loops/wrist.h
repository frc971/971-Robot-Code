#ifndef FRC971_CONTROL_LOOPS_WRIST_H_
#define FRC971_CONTROL_LOOPS_WRIST_H_

#include <memory>

#include "aos/common/control_loop/ControlLoop.h"
#include "frc971/control_loops/state_feedback_loop.h"
#include "frc971/control_loops/wrist_motor.q.h"
#include "frc971/control_loops/wrist_motor_plant.h"

namespace frc971 {
namespace control_loops {

namespace testing {
class WristTest_RezeroWithMissingPos_Test;
class WristTest_DisableGoesUninitialized_Test;
class WristTest_NoWindup_Test;
class WristTest_NoWindupPositive_Test;
class WristTest_NoWindupNegative_Test;
};

class WristMotor;

class WristMotor
    : public aos::control_loops::ControlLoop<control_loops::WristLoop> {
 public:
  explicit WristMotor(
      control_loops::WristLoop *my_wrist = &control_loops::wrist);

 protected:
  virtual void RunIteration(
      const ::aos::control_loops::Goal *goal,
      const control_loops::WristLoop::Position *position,
      ::aos::control_loops::Output *output,
      ::aos::control_loops::Status *status);

 private:
  // Friend the test classes for acces to the internal state.
  friend class testing::WristTest_RezeroWithMissingPos_Test;
  friend class testing::WristTest_DisableGoesUninitialized_Test;
  friend class testing::WristTest_NoWindupPositive_Test;
  friend class testing::WristTest_NoWindupNegative_Test;
  friend class WristStateFeedbackLoop;

  // Fetches and locally caches the latest set of constants.
  bool FetchConstants();

  // Clips the goal to be inside the limits and returns the clipped goal.
  // Requires the constants to have already been fetched.
  double ClipGoal(double goal) const;

  // This class implements the CapU function correctly given all the extra
  // information that we know about from the wrist motor.
  class WristStateFeedbackLoop : public StateFeedbackLoop<2, 1, 1> {
   public:
    WristStateFeedbackLoop(StateFeedbackLoop<2, 1, 1> loop,
                           WristMotor *wrist_motor)
        : StateFeedbackLoop<2, 1, 1>(loop),
          wrist_motor_(wrist_motor) {
    }

    // Caps U, but this time respects the state of the wrist as well.
    virtual void CapU();
   private:
    WristMotor *wrist_motor_;
  };

  // The state feedback control loop to talk to.
  ::std::unique_ptr<WristStateFeedbackLoop> loop_;

  // Enum to store the state of the internal zeroing state machine.
  enum State {
    UNINITIALIZED,
    MOVING_OFF,
    ZEROING,
    READY,
    ESTOP
  };

  // Internal state for zeroing.
  State state_;

  // Missed position packet count.
  int error_count_;
  // Offset from the raw encoder value to the absolute angle.
  double zero_offset_;
  // Position that gets incremented when zeroing the wrist to slowly move it to
  // the hall effect sensor.
  double zeroing_position_;
  // Last position at which the hall effect sensor was off.
  double last_off_position_;

  // Local cache of the wrist geometry constants.
  double horizontal_lower_limit_;
  double horizontal_upper_limit_;
  double horizontal_hall_effect_start_angle_;
  double horizontal_zeroing_speed_;

  DISALLOW_COPY_AND_ASSIGN(WristMotor);
};

}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_WRIST_H_
