#include "frc971/input/sensor_unpacker.h"

#include <arpa/inet.h>
#include <math.h>

#include "aos/common/inttypes.h"

#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/control_loops/wrist/wrist_motor.q.h"
#include "frc971/control_loops/angle_adjust/angle_adjust_motor.q.h"
#include "frc971/control_loops/index/index_motor.q.h"
#include "frc971/control_loops/shooter/shooter_motor.q.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using ::frc971::control_loops::drivetrain;
using ::frc971::control_loops::wrist;
using ::frc971::control_loops::angle_adjust;
using ::frc971::control_loops::shooter;
using ::frc971::control_loops::index_loop;

namespace frc971 {
namespace {

inline double drivetrain_translate(int32_t in) {
  // TODO(2013) fix the math
  return static_cast<double>(in) / (256.0 * 4.0 * 44.0 / 32.0) *
      (3.5 * 2.54 / 100.0 * M_PI);
}

inline double wrist_translate(int32_t in) {
  return -static_cast<double>(in) / (256.0 /*cpr*/ * 4.0 /*quad*/) *
      (14.0 / 50.0 * 20.0 / 84.0) /*gears*/ * (2 * M_PI);
}

inline double angle_adjust_translate(int32_t in) {
  static const double kCableDiameter = 0.060;
  return -static_cast<double>(in) / (256.0 /*cpr*/ * 2.0 /*2x*/) *
      ((0.75 + kCableDiameter) / (16.61125 + kCableDiameter)) /*pulleys*/ *
      (2 * M_PI);
}

inline double shooter_translate(int32_t in) {
 return -static_cast<double>(in) / (32.0 /*cpr*/ * 4.0 /*quad*/) *
      (15.0 / 34.0) /*gears*/ * (2 * M_PI);
}

inline double index_translate(int32_t in) {
  return -static_cast<double>(in) / (128.0 /*cpr*/ * 2.0 /*2x*/) *
      (1.0) /*gears*/ * (2 * M_PI);
}

}  // namespace

SensorUnpacker::SensorUnpacker() {}

void SensorUnpacker::UnpackFrom(sensor_values *values) {
  for (size_t i = 0;
       i < sizeof(values->encoders) / sizeof(values->encoders[0]); ++i) {
    values->encoders[i] = ntohl(values->encoders[i]);
  }

  // TODO(aschuh): Convert to meters.
  const double left_encoder = drivetrain_translate(
      values->drive_left_encoder);
  const double right_encoder = drivetrain_translate(
      values->drive_right_encoder);
  drivetrain.position.MakeWithBuilder()
      .left_encoder(left_encoder)
      .right_encoder(right_encoder)
      .Send();

  wrist.position.MakeWithBuilder()
      .pos(wrist_translate(values->wrist_position))
      .hall_effect(!values->wrist_hall_effect)
      .calibration(wrist_translate(values->wrist_edge_position))
      .Send();

  angle_adjust.position.MakeWithBuilder()
      .angle(angle_adjust_translate(values->angle_adjust_position))
      .bottom_hall_effect(!values->angle_adjust_bottom_hall_effect)
      .middle_hall_effect(!values->angle_adjust_middle_hall_effect && false)
      .bottom_calibration(angle_adjust_translate(
              values->angle_adjust_bottom_edge_position))
      .middle_calibration(angle_adjust_translate(
             values->angle_adjust_middle_edge_position))
      .Send();

  shooter.position.MakeWithBuilder()
      .position(shooter_translate(values->shooter_encoder))
      .Send();

  index_loop.position.MakeWithBuilder()
      .index_position(index_translate(values->index_encoder))
      .top_disc_detect(!values->top_disc)
      .top_disc_posedge_count(values->top_disc_posedge_count)
      .top_disc_posedge_position(index_translate(
              values->top_disc_posedge_position))
      .top_disc_negedge_count(values->top_disc_negedge_count)
      .top_disc_negedge_position(index_translate(
              values->top_disc_negedge_position))
      .bottom_disc_detect(!values->bottom_disc)
      .bottom_disc_posedge_count(values->bottom_disc_posedge_count)
      .bottom_disc_negedge_count(values->bottom_disc_negedge_count)
      .bottom_disc_negedge_wait_position(index_translate(
              values->bottom_disc_negedge_wait_position))
      .bottom_disc_negedge_wait_count(values->bottom_disc_negedge_wait_count)
      .Send();
}

}  // namespace frc971
