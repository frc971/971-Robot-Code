#ifndef Y2015_BOT3_CONTROL_LOOPS_ELEVATOR_H_
#define Y2015_BOT3_CONTROL_LOOPS_ELEVATOR_H_

#include <memory>

#include "aos/common/controls/control_loop.h"
#include "aos/common/util/trapezoid_profile.h"

#include "frc971/control_loops/state_feedback_loop.h"
#include "y2015_bot3/control_loops/elevator/elevator.q.h"

namespace y2015_bot3 {
namespace control_loops {
namespace testing {
class ElevatorTest_DisabledGoalTest_Test;
class ElevatorTest_ElevatorGoalPositiveWindupTest_Test;
class ElevatorTest_ElevatorGoalNegativeWindupTest_Test;
class ElevatorTest_PositiveRunawayProfileTest_Test;
class ElevatorTest_NegativeRunawayProfileTest_Test;
}  // namespace testing

// Constants
constexpr double kZeroingVoltage = 4.0;
// TODO(austin): Slow down the zeroing velocity.
constexpr double kZeroingVelocity = 0.05;
constexpr double kZeroingSlowVelocity = 0.01;

// Game pieces
constexpr double kToteHeight = 0.3;

// Gearing
constexpr double kElevEncoderRatio = 18.0 / 48.0;
constexpr double kElevChainReduction = 16.0 / 22.0;
const int kElevGearboxOutputPulleyTeeth = 22;
constexpr double kElevGearboxOutputPitch = 0.25 * 0.0254; // 25 pitch chain.

constexpr double kElevGearboxOutputRadianDistance =
    kElevGearboxOutputPulleyTeeth * kElevGearboxOutputPitch / (2.0 * M_PI);

// Limits
constexpr double kElevLowerHardLimit = -0.010;
constexpr double kElevUpperHardLimit = 1.387;
constexpr double kElevLowerLimit = 0.010;
constexpr double kElevUpperLimit = 1.350;

// Zeroing
namespace {
constexpr double kHallEffectPosition = 0.025;
}  // namespace
// End constants

class SimpleCappedStateFeedbackLoop : public StateFeedbackLoop<3, 1, 1> {
 public:
  SimpleCappedStateFeedbackLoop(StateFeedbackLoop<3, 1, 1> &&loop)
      : StateFeedbackLoop<3, 1, 1>(::std::move(loop)), max_voltage_(12.0) {}

  void set_max_voltage(double max_voltage) {
    max_voltage_ = ::std::max(0.0, ::std::min(12.0, max_voltage));
  }

  void CapU() override;

  // Returns the amount to change the position goal in order to no longer
  // saturate the controller.
  double UnsaturateOutputGoalChange();

 private:
  double max_voltage_;
};

// The GlitchFilter filters out single cycle changes in order to filter out
// sensor noise.  It also captures the sensor value at the time that the
// original transition happens (not the de-bounced transition) to remove delay
// caused by this filter.
class GlitchFilter {
 public:
  void Reset(bool hall_effect) {
    accepted_value_ = hall_effect;
    posedge_ = false;
    negedge_ = false;
    count_ = 0;
  }

  // Updates the filter state with new observations.
  void Update(bool hall_effect, double encoder);

  // Returns a debounced hall effect value.
  bool filtered_value() const { return accepted_value_; }

  // Returns true if last cycle had a posedge.
  bool posedge() const { return posedge_; }
  // Returns the encoder value across the last posedge.
  double posedge_value() const { return posedge_value_; }

  // Returns true if last cycle had a negedge.
  bool negedge() const { return negedge_; }
  // Returns the encoder value across the last negedge.
  double negedge_value() const { return negedge_value_; }

 private:
  int count_ = 0;
  bool accepted_value_ = false;
  bool posedge_ = false;
  bool negedge_ = false;
  double posedge_value_ = 0;
  double negedge_value_ = 0;

  double first_encoder_ = 0;
  double last_encoder_ = 0;
};

class Elevator
    : public aos::controls::ControlLoop<control_loops::ElevatorQueue> {
 public:
  explicit Elevator(control_loops::ElevatorQueue *elevator_queue =
                        &control_loops::elevator_queue);

  bool CheckZeroed();

  // Getter for the current elevator positions with zeroed offset.
  double current_position();

  double GetZeroingVelocity();

  enum State {
    // Waiting to receive data before doing anything.
    UNINITIALIZED = 0,
    // Moving the elevator to find an index pulse.
    ZEROING = 2,
    // All good!
    RUNNING = 3,
    // Internal error caused the elevator to abort.
    ESTOP = 4,
  };

  State state() const { return state_; }

 protected:
  void RunIteration(const control_loops::ElevatorQueue::Goal *goal,
                    const control_loops::ElevatorQueue::Position *position,
                    control_loops::ElevatorQueue::Output *output,
                    control_loops::ElevatorQueue::Status *status) override;

 private:
  friend class testing::ElevatorTest_DisabledGoalTest_Test;
  friend class testing::ElevatorTest_ElevatorGoalPositiveWindupTest_Test;
  friend class testing::ElevatorTest_ElevatorGoalNegativeWindupTest_Test;
  friend class testing::ElevatorTest_PositiveRunawayProfileTest_Test;
  friend class testing::ElevatorTest_NegativeRunawayProfileTest_Test;

  void SetOffset(double offset);

  // Returns the current elevator zeroing velocity.
  double FindZeroingVelocity();

  // Corrects the Observer with the current position.
  void Correct();

  double UseUnlessZero(double target_value, double default_value);

  // The state feedback control loop or loops to talk to.
  ::std::unique_ptr<SimpleCappedStateFeedbackLoop> loop_;

  // Offsets from the encoder position to the absolute position.  Add these to
  // the encoder position to get the absolute position.
  double offset_ = 0.0;

  bool zeroed_ = false;
  State state_ = State::UNINITIALIZED;

  // Variables for detecting the zeroing hall effect edge.
  bool zeroing_hall_effect_edge_detector_initialized_ = false;
  bool zeroing_hall_effect_previous_reading_ = false;

  // Current velocity to move at while zeroing.
  double zeroing_velocity_ = 0.0;

  // The goals for the elevator.
  double goal_ = 0.0;
  double goal_velocity_ = 0.0;

  control_loops::ElevatorQueue::Position current_position_;

  aos::util::TrapezoidProfile profile_;
  GlitchFilter glitch_filter_;
};

}  // namespace control_loops
}  // namespace y2015_bot3

#endif  // Y2015_BOT3_CONTROL_LOOPS_ELEVATOR_H_
