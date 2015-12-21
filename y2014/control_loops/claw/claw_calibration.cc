#include "y2014/control_loops/claw/claw.q.h"
#include "frc971/control_loops/control_loops.q.h"

#include "aos/linux_code/init.h"
#include "y2014/constants.h"

namespace y2014 {

typedef constants::Values::Claws Claws;
using ::frc971::HallEffectStruct;

class Sensor {
 public:
  Sensor(const double start_position,
         const HallEffectStruct &initial_hall_effect)
      : start_position_(start_position),
        last_hall_effect_(initial_hall_effect),
        last_posedge_count_(initial_hall_effect.posedge_count),
        last_negedge_count_(initial_hall_effect.negedge_count) {
    last_on_min_position_ = start_position;
    last_on_max_position_ = start_position;
    last_off_min_position_ = start_position;
    last_off_max_position_ = start_position;
  }

  bool DoGetPositionOfEdge(
      const ::y2014::control_loops::HalfClawPosition &claw_position,
      const HallEffectStruct &hall_effect, Claws::AnglePair *limits) {
    bool print = false;

    if (hall_effect.posedge_count != last_posedge_count_) {
      const double avg_off_position =
          (last_off_min_position_ + last_off_max_position_) / 2.0;
      if (hall_effect.posedge_value < avg_off_position) {
        printf("Posedge upper current %f posedge %f avg_off %f [%f, %f]\n",
               claw_position.position, hall_effect.posedge_value,
               avg_off_position, last_off_min_position_,
               last_off_max_position_);
        limits->upper_decreasing_angle =
            hall_effect.posedge_value - start_position_;
      } else {
        printf("Posedge lower current %f posedge %f avg_off %f [%f, %f]\n",
               claw_position.position, hall_effect.posedge_value,
               avg_off_position, last_off_min_position_,
               last_off_max_position_);
        limits->lower_angle =
            hall_effect.posedge_value - start_position_;
      }
      print = true;
    }
    if (hall_effect.negedge_count != last_negedge_count_) {
      const double avg_on_position =
          (last_on_min_position_ + last_on_max_position_) / 2.0;
      if (hall_effect.negedge_value > avg_on_position) {
        printf("Negedge upper current %f negedge %f last_on %f [%f, %f]\n",
               claw_position.position, hall_effect.negedge_value,
               avg_on_position, last_on_min_position_,
               last_on_max_position_);
        limits->upper_angle =
            hall_effect.negedge_value - start_position_;
      } else {
        printf("Negedge lower current %f negedge %f last_on %f [%f, %f]\n",
               claw_position.position, hall_effect.negedge_value,
               avg_on_position, last_on_min_position_,
               last_on_max_position_);
        limits->lower_decreasing_angle =
            hall_effect.negedge_value - start_position_;
      }
      print = true;
    }

    if (hall_effect.current) {
      if (!last_hall_effect_.current) {
        last_on_min_position_ = last_on_max_position_ = claw_position.position;
      } else {
        last_on_min_position_ =
            ::std::min(claw_position.position, last_on_min_position_);
        last_on_max_position_ =
            ::std::max(claw_position.position, last_on_max_position_);
      }
    } else {
      if (last_hall_effect_.current) {
        last_off_min_position_ = last_off_max_position_ =
            claw_position.position;
      } else {
        last_off_min_position_ =
            ::std::min(claw_position.position, last_off_min_position_);
        last_off_max_position_ =
            ::std::max(claw_position.position, last_off_max_position_);
      }
    }

    last_hall_effect_ = hall_effect;
    last_posedge_count_ = hall_effect.posedge_count;
    last_negedge_count_ = hall_effect.negedge_count;

    return print;
  }

 private:
  const double start_position_;
  HallEffectStruct last_hall_effect_;
  int32_t last_posedge_count_;
  int32_t last_negedge_count_;
  double last_on_min_position_;
  double last_off_min_position_;
  double last_on_max_position_;
  double last_off_max_position_;
};

class ClawSensors {
 public:
  ClawSensors(
      const double start_position,
      const ::y2014::control_loops::HalfClawPosition &initial_claw_position)
      : start_position_(start_position),
        front_(start_position, initial_claw_position.front),
        calibration_(start_position, initial_claw_position.calibration),
        back_(start_position, initial_claw_position.back) {}

  bool GetPositionOfEdge(
      const ::y2014::control_loops::HalfClawPosition &claw_position,
      Claws::Claw *claw) {
    bool print = false;
    if (front_.DoGetPositionOfEdge(claw_position,
                                   claw_position.front, &claw->front)) {
      print = true;
    } else if (calibration_.DoGetPositionOfEdge(claw_position,
                                                claw_position.calibration,
                                                &claw->calibration)) {
      print = true;
    } else if (back_.DoGetPositionOfEdge(claw_position,
                                         claw_position.back, &claw->back)) {
      print = true;
    }

    double position = claw_position.position - start_position_;

    if (position > claw->upper_limit) {
      claw->upper_hard_limit = claw->upper_limit = position;
      print = true;
    }
    if (position < claw->lower_limit) {
      claw->lower_hard_limit = claw->lower_limit = position;
      print = true;
    }
    return print;
  }

 private:
  const double start_position_;
  Sensor front_;
  Sensor calibration_;
  Sensor back_;
};

int Main() {
  ::y2014::control_loops::claw_queue.position.FetchNextBlocking();

  const double top_start_position =
      ::y2014::control_loops::claw_queue.position->top.position;
  const double bottom_start_position =
      ::y2014::control_loops::claw_queue.position->bottom.position;

  ClawSensors top(top_start_position,
                  ::y2014::control_loops::claw_queue.position->top);
  ClawSensors bottom(bottom_start_position,
                     ::y2014::control_loops::claw_queue.position->bottom);

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

  ::y2014::control_loops::ClawQueue::Position last_position =
      *::y2014::control_loops::claw_queue.position;

  while (true) {
    ::y2014::control_loops::claw_queue.position.FetchNextBlocking();
    bool print = false;
    if (top.GetPositionOfEdge(::y2014::control_loops::claw_queue.position->top,
                              &limits.upper_claw)) {
      print = true;
      printf("Got an edge on the upper claw\n");
    }
    if (bottom.GetPositionOfEdge(
            ::y2014::control_loops::claw_queue.position->bottom,
            &limits.lower_claw)) {
      print = true;
      printf("Got an edge on the lower claw\n");
    }
    const double top_position =
        ::y2014::control_loops::claw_queue.position->top.position -
        top_start_position;
    const double bottom_position =
        ::y2014::control_loops::claw_queue.position->bottom.position -
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

    last_position = *::y2014::control_loops::claw_queue.position;
  }
  return 0;
}

}  // namespace y2014

int main() {
  ::aos::Init();
  int returnvalue = ::y2014::Main();
  ::aos::Cleanup();
  return returnvalue;
}
