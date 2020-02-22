#ifndef Y2020_CONTROL_LOOPS_SHOOTER_FLYWHEEL_CONTROLLER_H_
#define Y2020_CONTROL_LOOPS_SHOOTER_FLYWHEEL_CONTROLLER_H_

#include <memory>

#include "aos/controls/control_loop.h"
#include "aos/time/time.h"
#include "frc971/control_loops/state_feedback_loop.h"
#include "y2020/control_loops/superstructure/accelerator/integral_accelerator_plant.h"
#include "y2020/control_loops/superstructure/finisher/integral_finisher_plant.h"
#include "y2020/control_loops/superstructure/superstructure_status_generated.h"

namespace y2020 {
namespace control_loops {
namespace superstructure {
namespace shooter {

// Handles the velocity control of each flywheel.
class FlywheelController {
 public:
  FlywheelController(StateFeedbackLoop<3, 1, 1> &&loop);

  // Sets the velocity goal in radians/sec
  void set_goal(double angular_velocity_goal);
  // Sets the current encoder position in radians
  void set_position(double current_position);

  // Populates the status structure.
  flatbuffers::Offset<FlywheelControllerStatus> SetStatus(
      flatbuffers::FlatBufferBuilder *fbb);

  // Returns the control loop calculated voltage.
  double voltage() const;

  // Returns the instantaneous velocity.
  double velocity() const { return loop_->X_hat(1, 0); }

  // Executes the control loop for a cycle.
  void Update(bool disabled);

 private:
  // The current sensor measurement.
  Eigen::Matrix<double, 1, 1> Y_;
  // The control loop.
  ::std::unique_ptr<StateFeedbackLoop<3, 1, 1>> loop_;

  // History array for calculating a filtered angular velocity.
  static constexpr int kHistoryLength = 10;
  ::std::array<double, kHistoryLength> history_;
  ptrdiff_t history_position_ = 0;
  double last_goal_ = 0;

  DISALLOW_COPY_AND_ASSIGN(FlywheelController);
};

}  // namespace shooter
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2020

#endif  // Y2020_CONTROL_LOOPS_SHOOTER_FLYWHEEL_CONTROLLER_H_
