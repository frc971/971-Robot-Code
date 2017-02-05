#ifndef Y2016_BOT3_CONTROL_LOOPS_INTAKE_INTAKE_H_
#define Y2016_BOT3_CONTROL_LOOPS_INTAKE_INTAKE_H_

#include <memory>

#include "aos/common/controls/control_loop.h"
#include "aos/common/util/trapezoid_profile.h"
#include "frc971/control_loops/state_feedback_loop.h"

#include "frc971/zeroing/zeroing.h"
#include "y2016_bot3/control_loops/intake/intake.q.h"
#include "y2016_bot3/control_loops/intake/intake_controls.h"

namespace y2016_bot3 {
namespace constants {
static const int kZeroingSampleSize = 200;

// Ratios for our subsystems.
// TODO(constants): Update these.
static constexpr double kIntakeEncoderRatio = 18.0 / 72.0 * 15.0 / 48.0;

static constexpr double kIntakePotRatio = 15.0 / 48.0;

// Difference in radians between index pulses.
static constexpr double kIntakeEncoderIndexDifference =
    2.0 * M_PI * kIntakeEncoderRatio;

// Subsystem motion ranges, in whatever units that their respective queues say
// the use.
// TODO(constants): Update these.
static constexpr ::frc971::constants::Range kIntakeRange{// Lower hard stop
                                                         -0.4,
                                                         // Upper hard stop
                                                         2.90,
                                                         // Lower soft stop
                                                         -0.28,
                                                         // Uppper soft stop
                                                         2.77};

struct IntakeZero {
  static constexpr double pot_offset = 5.462409 + 0.333162;
  static constexpr ::frc971::constants::PotAndIndexPulseZeroingConstants
      zeroing{kZeroingSampleSize, kIntakeEncoderIndexDifference,
              0.148604 - 0.291240, 0.3};
};
}  // namespace constants
namespace control_loops {
namespace intake {
namespace testing {
class IntakeTest_RespectsRange_Test;
class IntakeTest_DisabledGoalTest_Test;
class IntakeTest_IntakeZeroingErrorTest_Test;
class IntakeTest_UpperHardstopStartup_Test;
class IntakeTest_DisabledWhileZeroingHigh_Test;
class IntakeTest_DisabledWhileZeroingLow_Test;
}

// TODO(Adam): Implement this class and delete it from here.
class LimitChecker {
 public:
  LimitChecker(IntakeArm *intake) : intake_(intake) {}
  void UpdateGoal(double intake_angle_goal);

 private:
  IntakeArm *intake_;
};

class Intake : public ::aos::controls::ControlLoop<control_loops::IntakeQueue> {
 public:
  explicit Intake(
      control_loops::IntakeQueue *my_intake = &control_loops::intake_queue);

  static constexpr double kZeroingVoltage = 6.0;
  static constexpr double kOperatingVoltage = 12.0;

  // This is the large scale movement tolerance.
  static constexpr double kLooseTolerance = 0.05;

  // This is the small scale movement tolerance.
  static constexpr double kTightTolerance = 0.03;

  static constexpr double kIntakeUpAngle = M_PI / 2;

  static constexpr double kIntakeDownAngle = 0.0;

  static constexpr double kIntakeMiddleAngle =
      (kIntakeUpAngle + kIntakeDownAngle) / 2;

  enum State {
    // Wait for all the filters to be ready before starting the initialization
    // process.
    UNINITIALIZED = 0,

    // We now are ready to decide how to zero.  Decide what to do once we are
    // enabled.
    DISABLED_INITIALIZED = 1,

    ZERO_LOWER_INTAKE = 2,

    ZERO_LIFT_INTAKE = 3,
    // Run, but limit power to zeroing voltages.
    SLOW_RUNNING = 12,
    // Run with full power.
    RUNNING = 13,
    // Internal error caused the intake to abort.
    ESTOP = 16,
  };

  bool IsRunning() const {
    return (state_ == SLOW_RUNNING || state_ == RUNNING);
  }

  State state() const { return state_; }

 protected:
  void RunIteration(const control_loops::IntakeQueue::Goal *unsafe_goal,
                    const control_loops::IntakeQueue::Position *position,
                    control_loops::IntakeQueue::Output *output,
                    control_loops::IntakeQueue::Status *status) override;

 private:
  friend class testing::IntakeTest_DisabledGoalTest_Test;
  friend class testing::IntakeTest_IntakeZeroingErrorTest_Test;
  friend class testing::IntakeTest_RespectsRange_Test;
  friend class testing::IntakeTest_UpperHardstopStartup_Test;
  friend class testing::IntakeTest_DisabledWhileZeroingHigh_Test;
  friend class testing::IntakeTest_DisabledWhileZeroingLow_Test;
  IntakeArm intake_;

  State state_ = UNINITIALIZED;
  State last_state_ = UNINITIALIZED;

  float last_intake_angle_ = 0.0;
  LimitChecker limit_checker_;
  // Returns true if the profile has finished, and the joint is within the
  // specified tolerance.
  bool IsIntakeNear(double tolerance);

  DISALLOW_COPY_AND_ASSIGN(Intake);
};

}  // namespace intake
}  // namespace control_loops
}  // namespace y2016_bot3

#endif  // Y2016_BOT3_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_
