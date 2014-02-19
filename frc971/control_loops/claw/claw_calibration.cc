#include "frc971/control_loops/claw/claw.q.h"
#include "frc971/control_loops/control_loops.q.h"

#include "aos/linux_code/init.h"
#include "frc971/constants.h"

namespace frc971 {

typedef constants::Values::Claws Claws;

bool DoGetPositionOfEdge(
    const double start_position,
    const control_loops::HalfClawPosition &last_claw_position,
    const control_loops::HalfClawPosition &claw_position,
    const HallEffectStruct &last_hall_effect,
    const HallEffectStruct &hall_effect,
    Claws::AnglePair *limits) {

  if (hall_effect.posedge_count != last_hall_effect.posedge_count) {
    if (claw_position.posedge_value < last_claw_position.position) {
      limits->upper_angle = claw_position.posedge_value - start_position;
    } else {
      limits->lower_decreasing_angle =
          claw_position.posedge_value - start_position;
    }
    return true;
  }
  if (hall_effect.negedge_count != last_hall_effect.negedge_count) {
    if (claw_position.negedge_value > last_claw_position.position) {
      limits->upper_decreasing_angle =
          claw_position.negedge_value - start_position;
    } else {
      limits->lower_angle = claw_position.negedge_value - start_position;
    }
    return true;
  }

  return false;
}

bool GetPositionOfEdge(
    const double start_position,
    const control_loops::HalfClawPosition &last_claw_position,
    const control_loops::HalfClawPosition &claw_position, Claws::Claw *claw) {

  if (DoGetPositionOfEdge(start_position, last_claw_position, claw_position,
                          last_claw_position.front, claw_position.front,
                          &claw->front)) {
    return true;
  }
  if (DoGetPositionOfEdge(start_position, last_claw_position, claw_position,
                          last_claw_position.calibration,
                          claw_position.calibration, &claw->calibration)) {
    return true;
  }
  if (DoGetPositionOfEdge(start_position, last_claw_position, claw_position,
                          last_claw_position.back, claw_position.back,
                          &claw->back)) {
    return true;
  }

  double position = claw_position.position - start_position;

  if (position > claw->upper_limit) {
    claw->upper_hard_limit = claw->upper_limit = position;
    return true;
  }
  if (position < claw->lower_limit) {
    claw->lower_hard_limit = claw->lower_limit = position;
    return true;
  }
  return false;
}

int Main() {
  while (!control_loops::claw_queue_group.position.FetchNextBlocking());

  const double top_start_position =
      control_loops::claw_queue_group.position->top.position;
  const double bottom_start_position =
      control_loops::claw_queue_group.position->bottom.position;

  Claws limits;

  limits.claw_zeroing_off_speed = 0.5;
  limits.claw_zeroing_speed = 0.1;
  limits.claw_zeroing_separation = 0.1;

  // claw separation that would be considered a collision
  limits.claw_min_separation = 0.0;
  limits.claw_max_separation = 0.0;

  // We should never get closer/farther than these.
  limits.soft_min_separation = 0.0;
  limits.soft_max_separation = 0.0;

  limits.upper_claw.lower_hard_limit = 0.0;
  limits.upper_claw.upper_hard_limit = 0.0;
  limits.upper_claw.lower_limit = 0.0;
  limits.upper_claw.upper_limit = 0.0;
  limits.upper_claw.front.lower_angle = 0.0;
  limits.upper_claw.front.upper_angle = 0.0;
  limits.upper_claw.front.lower_decreasing_angle = 0.0;
  limits.upper_claw.front.upper_decreasing_angle = 0.0;
  limits.upper_claw.calibration.lower_angle = 0.0;
  limits.upper_claw.calibration.upper_angle = 0.0;
  limits.upper_claw.calibration.lower_decreasing_angle = 0.0;
  limits.upper_claw.calibration.upper_decreasing_angle = 0.0;
  limits.upper_claw.back.lower_angle = 0.0;
  limits.upper_claw.back.upper_angle = 0.0;
  limits.upper_claw.back.lower_decreasing_angle = 0.0;
  limits.upper_claw.back.upper_decreasing_angle = 0.0;

  limits.lower_claw.lower_hard_limit = 0.0;
  limits.lower_claw.upper_hard_limit = 0.0;
  limits.lower_claw.lower_limit = 0.0;
  limits.lower_claw.upper_limit = 0.0;
  limits.lower_claw.front.lower_angle = 0.0;
  limits.lower_claw.front.upper_angle = 0.0;
  limits.lower_claw.front.lower_decreasing_angle = 0.0;
  limits.lower_claw.front.upper_decreasing_angle = 0.0;
  limits.lower_claw.calibration.lower_angle = 0.0;
  limits.lower_claw.calibration.upper_angle = 0.0;
  limits.lower_claw.calibration.lower_decreasing_angle = 0.0;
  limits.lower_claw.calibration.upper_decreasing_angle = 0.0;
  limits.lower_claw.back.lower_angle = 0.0;
  limits.lower_claw.back.upper_angle = 0.0;
  limits.lower_claw.back.lower_decreasing_angle = 0.0;
  limits.lower_claw.back.upper_decreasing_angle = 0.0;

  limits.claw_unimportant_epsilon = 0.01;
  limits.start_fine_tune_pos = -0.2;
  limits.max_zeroing_voltage = 4.0;

  control_loops::ClawGroup::Position last_position =
      *control_loops::claw_queue_group.position;

  while (true) {
    if (control_loops::claw_queue_group.position.FetchNextBlocking()) {
      bool print = false;
      if (GetPositionOfEdge(top_start_position, last_position.top,
                            control_loops::claw_queue_group.position->top,
                            &limits.upper_claw)) {
        print = true;
        LOG(DEBUG, "Got an edge on the upper claw\n");
      }
      if (GetPositionOfEdge(bottom_start_position, last_position.bottom,
                            control_loops::claw_queue_group.position->bottom,
                            &limits.lower_claw)) {
        print = true;
        LOG(DEBUG, "Got an edge on the lower claw\n");
      }
      const double top_position =
          control_loops::claw_queue_group.position->top.position -
          top_start_position;
      const double bottom_position =
          control_loops::claw_queue_group.position->bottom.position -
          bottom_start_position;
      const double separation = top_position - bottom_position;
      if (separation > limits.claw_max_separation) {
        limits.soft_max_separation = limits.claw_max_separation = separation;
        print = true;
      }
      if (separation < limits.claw_min_separation) {
        limits.soft_min_separation = limits.claw_min_separation = separation;
        print = true;
      }

      if (print) {
        printf("{%f,\n", limits.claw_zeroing_off_speed);
        printf("%f,\n", limits.claw_zeroing_speed);
        printf("%f,\n", limits.claw_zeroing_separation);
        printf("%f,\n", limits.claw_min_separation);
        printf("%f,\n", limits.claw_max_separation);
        printf("%f,\n", limits.soft_min_separation);
        printf("%f,\n", limits.soft_max_separation);
        printf(
            "{%f, %f, %f, %f, {%f, %f, %f, %f}, {%f, %f, %f, %f}, {%f, %f, %f, "
            "%f}},\n",
            limits.upper_claw.lower_hard_limit,
            limits.upper_claw.upper_hard_limit, limits.upper_claw.lower_limit,
            limits.upper_claw.upper_limit, limits.upper_claw.front.lower_angle,
            limits.upper_claw.front.upper_angle,
            limits.upper_claw.front.lower_decreasing_angle,
            limits.upper_claw.front.upper_decreasing_angle,
            limits.upper_claw.calibration.lower_angle,
            limits.upper_claw.calibration.upper_angle,
            limits.upper_claw.calibration.lower_decreasing_angle,
            limits.upper_claw.calibration.upper_decreasing_angle,
            limits.upper_claw.back.lower_angle,
            limits.upper_claw.back.upper_angle,
            limits.upper_claw.back.lower_decreasing_angle,
            limits.upper_claw.back.upper_decreasing_angle);

        printf(
            "{%f, %f, %f, %f, {%f, %f, %f, %f}, {%f, %f, %f, %f}, {%f, %f, %f, "
            "%f}},\n",
            limits.lower_claw.lower_hard_limit,
            limits.lower_claw.upper_hard_limit, limits.lower_claw.lower_limit,
            limits.lower_claw.upper_limit, limits.lower_claw.front.lower_angle,
            limits.lower_claw.front.upper_angle,
            limits.lower_claw.front.lower_decreasing_angle,
            limits.lower_claw.front.upper_decreasing_angle,
            limits.lower_claw.calibration.lower_angle,
            limits.lower_claw.calibration.upper_angle,
            limits.lower_claw.calibration.lower_decreasing_angle,
            limits.lower_claw.calibration.upper_decreasing_angle,
            limits.lower_claw.back.lower_angle,
            limits.lower_claw.back.upper_angle,
            limits.lower_claw.back.lower_decreasing_angle,
            limits.lower_claw.back.upper_decreasing_angle);
        printf("%f,  // claw_unimportant_epsilon\n",
               limits.claw_unimportant_epsilon);
        printf("%f,   // start_fine_tune_pos\n", limits.start_fine_tune_pos);
        printf("%f,\n", limits.max_zeroing_voltage);
        printf("}\n");
      }

      last_position = *control_loops::claw_queue_group.position;
    }
  }
  return 0;
}

}  // namespace frc971

int main() {
  ::aos::Init();
  int returnvalue = ::frc971::Main();
  ::aos::Cleanup();
  return returnvalue;
}
