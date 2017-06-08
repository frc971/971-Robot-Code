#include "y2017/control_loops/superstructure/vision_time_adjuster.h"

#include <chrono>

#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "y2017/control_loops/drivetrain/drivetrain_dog_motor_plant.h"
#include "y2017/control_loops/superstructure/superstructure.q.h"

namespace y2017 {
namespace control_loops {
namespace superstructure {

using ::aos::monotonic_clock;
namespace chrono = ::std::chrono;

namespace {

// Finds the two data points in the ring buffer between which the specified
// time falls. The results are stored in "before" and "after. If the specified
// time happens to co-incide perfectly with a data point, then "before" and
// "after" will both point to that data point.
template <typename Data, size_t buffer_size>
bool FindBeforeAndAfter(const aos::RingBuffer<Data, buffer_size> &data,
                        monotonic_clock::time_point time, Data *before,
                        Data *after) {
  // With no data, there's nothing we can do.
  if (data.size() == 0) return false;

  // If we only have one data point, we pretend that it's that data point.
  if (data.size() == 1) {
    *before = data[0];
    *after = data[0];
    return true;
  }

  // Find the two data points that match the two robots.
  for (size_t i = 0; i < data.size(); ++i) {
    if (data[i].time > time) {
      if (i == 0) {
        // If this if the first data point and it's already past the time stamp
        // we're looking for then we just use the first data point for both.
        *before = data[i];
      } else {
        *before = data[i - 1];
      }
      *after = data[i];
      return true;
    }
  }

  // We've hit the end of our data. Just fill both of them as the last data
  // point we have.
  *before = data[data.size() - 1];
  *after = data[data.size() - 1];
  return true;
}

double Interpolate(monotonic_clock::time_point before_time, double before_data,
                   monotonic_clock::time_point after_time, double after_data,
                   monotonic_clock::time_point current_time) {
  const double age_ratio =
      ::std::chrono::duration_cast<::std::chrono::duration<double>>(
          current_time - before_time)
          .count() /
      ::std::chrono::duration_cast<::std::chrono::duration<double>>(after_time -
                                                                    before_time)
          .count();
  return before_data * (1 - age_ratio) + after_data * age_ratio;
}

double ComputeColumnPosition(const VisionTimeAdjuster::ColumnAngle &position) {
  return position.turret;
}

double ComputeDrivetrainPosition(
    const VisionTimeAdjuster::DrivetrainAngle &position) {
  return (position.right - position.left) /
         (::y2017::control_loops::drivetrain::kRobotRadius * 2.0);
}

template <typename Data, size_t buffer_size>
bool ComputeAngle(const ::aos::RingBuffer<Data, buffer_size> &data,
                  monotonic_clock::time_point time,
                  ::std::function<double(const Data &)> compute_position,
                  double *result) {
  Data before;
  Data after;
  bool offset_is_valid = false;

  if (FindBeforeAndAfter(data, time, &before, &after)) {
    double position;
    double before_position = compute_position(before);
    double after_position = compute_position(after);
    if (before.time == after.time) {
      position = before_position;
    } else {
      position = Interpolate(before.time, before_position, after.time,
                             after_position, time);
    }
    offset_is_valid = true;
    *result = position;
  }

  return offset_is_valid;
}

}  // namespace

VisionTimeAdjuster::VisionTimeAdjuster() {}

void VisionTimeAdjuster::Tick(monotonic_clock::time_point monotonic_now,
                              double turret_position,
                              const vision::VisionStatus *vision_status) {
  // We have new column data, store it.
  column_data_.Push({.time = monotonic_now, .turret = turret_position});

  // If we have new drivetrain data, we store it.
  if (::frc971::control_loops::drivetrain_queue.status.FetchLatest()) {
    const auto &position = ::frc971::control_loops::drivetrain_queue.status;
    DrivetrainAngle new_position{.time = position->sent_time,
                                 .left = position->estimated_left_position,
                                 .right = position->estimated_right_position};
    drivetrain_data_.Push(new_position);
    most_recent_drivetrain_angle_ = ComputeDrivetrainPosition(new_position);
  }

  // If we have new vision data, compute the newest absolute angle at which the
  // target is.
  if (vision_status != nullptr && vision_status->image_valid) {
    monotonic_clock::time_point last_target_time(
        monotonic_clock::duration(vision_status->target_time));

    double column_angle = 0;
    double drivetrain_angle = 0;

    bool column_angle_is_valid = ComputeAngle<ColumnAngle>(
        column_data_, last_target_time, ComputeColumnPosition, &column_angle);
    bool drivetrain_angle_is_valid = ComputeAngle<DrivetrainAngle>(
        drivetrain_data_, last_target_time, ComputeDrivetrainPosition,
        &drivetrain_angle);

    if (column_angle_is_valid && drivetrain_angle_is_valid) {
      LOG(INFO, "Accepting Vision angle of %f, age %f\n",
          most_recent_vision_angle_,
          chrono::duration_cast<chrono::duration<double>>(
              monotonic_now - last_target_time).count());
      most_recent_vision_reading_ = vision_status->angle;
      most_recent_vision_angle_ =
          vision_status->angle + column_angle + drivetrain_angle;
      most_recent_vision_time_ = monotonic_now;
    }
  }

  goal_ = most_recent_vision_angle_ - most_recent_drivetrain_angle_;
  LOG(INFO, "Vision angle %f drivetrain %f\n", most_recent_vision_angle_,
      most_recent_drivetrain_angle_);

  // Now, update the vision valid flag to tell us if we have a valid vision
  // angle within the last seven seconds.
  if (monotonic_now < most_recent_vision_time_ + chrono::seconds(7)) {
    valid_ = true;
  } else {
    valid_ = false;
  }
}

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2017
