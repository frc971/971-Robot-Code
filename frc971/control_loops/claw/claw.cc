#include "frc971/control_loops/claw/claw.h"

#include <stdio.h>

#include <algorithm>

#include "aos/common/control_loop/control_loops.q.h"
#include "aos/common/logging/logging.h"

#include "frc971/constants.h"
#include "frc971/control_loops/claw/top_claw_motor_plant.h"
#include "frc971/control_loops/claw/bottom_claw_motor_plant.h"

// Zeroing plan.
// There are 2 types of zeros.  Enabled and disabled ones.
// Disabled ones are only valid during auto mode, and can be used to speed up
// the enabled zero process.  We need to re-zero during teleop in case the auto
// zero was poor and causes us to miss all our shots.
//
// We need to be able to zero manually while disabled by moving the joint over
// the zeros.
// Zero on the down edge when disabled (gravity in the direction of motion)
//
// When enabled, zero on the up edge (gravity opposing the direction of motion)
// The enabled sequence needs to work as follows.  We can crash the claw if we
// bring them too close to each other or too far from each other.  The only safe
// thing to do is to move them in unison.
//
// Start by moving them both towards the front of the bot to either find either
// the middle hall effect on either jaw, or the front hall effect on the bottom
// jaw.  Any edge that isn't the desired edge will provide an approximate edge
// location that can be used for the fine tuning step.
// Once an edge is found on the front claw, move back the other way with both
// claws until an edge is found for the other claw.
// Now that we have an approximate zero, we can robustify the limits to keep
// both claws safe.  Then, we can move both claws to a position that is the
// correct side of the zero and go zero.

// Valid region plan.
// Difference between the arms has a range, and the values of each arm has a range.
// If a claw runs up against a static limit, don't let the goal change outside
// the limit.
// If a claw runs up against a movable limit, move both claws outwards to get
// out of the condition.

namespace frc971 {
namespace control_loops {

ClawMotor::ClawMotor(control_loops::ClawLoop *my_claw)
    : aos::control_loops::ControlLoop<control_loops::ClawLoop>(my_claw),
      zeroed_joint_(MakeTopClawLoop()) {
  {
    using ::frc971::constants::GetValues;
    ZeroedJoint<1>::ConfigurationData config_data;

    config_data.lower_limit = GetValues().claw_lower_limit;
    config_data.upper_limit = GetValues().claw_upper_limit;
    config_data.hall_effect_start_angle[0] =
        GetValues().claw_hall_effect_start_angle;
    config_data.zeroing_off_speed = GetValues().claw_zeroing_off_speed;
    config_data.zeroing_speed = GetValues().claw_zeroing_speed;

    config_data.max_zeroing_voltage = 5.0;
    config_data.deadband_voltage = 0.0;

    zeroed_joint_.set_config_data(config_data);
  }
}

// Positive angle is up, and positive power is up.
void ClawMotor::RunIteration(const control_loops::ClawLoop::Goal *goal,
                             const control_loops::ClawLoop::Position *position,
                             control_loops::ClawLoop::Output *output,
                             ::aos::control_loops::Status *status) {

  // Disable the motors now so that all early returns will return with the
  // motors disabled.
  if (output) {
    output->top_claw_voltage = 0;
    output->bottom_claw_voltage = 0;
    output->intake_voltage = 0;
  }

  ZeroedJoint<1>::PositionData transformed_position;
  ZeroedJoint<1>::PositionData *transformed_position_ptr =
      &transformed_position;
  if (!position) {
    transformed_position_ptr = NULL;
  } else {
    transformed_position.position = position->top_position;
    transformed_position.hall_effects[0] = position->top_calibration_hall_effect;
    transformed_position.hall_effect_positions[0] = position->top_posedge_value;
  }

  const double voltage =
      zeroed_joint_.Update(transformed_position_ptr, output != NULL,
                           goal->bottom_angle + goal->seperation_angle, 0.0);

  if (position) {
    LOG(DEBUG, "pos: %f hall: %s absolute: %f\n", position->top_position,
        position->top_calibration_hall_effect ? "true" : "false",
        zeroed_joint_.absolute_position());
  }

  if (output) {
    output->top_claw_voltage = voltage;
  }
  status->done = ::std::abs(zeroed_joint_.absolute_position() -
                            goal->bottom_angle - goal->seperation_angle) < 0.004;
}

}  // namespace control_loops
}  // namespace frc971
