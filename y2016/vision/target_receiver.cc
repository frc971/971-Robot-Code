#include <netdb.h>
#include <stdlib.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <chrono>
#include <limits>
#include <memory>
#include <thread>
#include <vector>

#include "aos/events/event_loop.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/logging/logging.h"
#include "aos/mutex/mutex.h"
#include "aos/time/time.h"
#include "aos/vision/events/udp.h"
#include "frc971/control_loops/drivetrain/drivetrain_status_generated.h"
#include "y2016/constants.h"
#include "y2016/vision/stereo_geometry.h"
#include "y2016/vision/vision_data.pb.h"
#include "y2016/vision/vision_generated.h"

namespace y2016 {
namespace vision {

namespace chrono = ::std::chrono;
using ::aos::monotonic_clock;

::aos::vision::Vector<2> CreateCenterFromTarget(double lx, double ly, double rx,
                                                double ry) {
  return ::aos::vision::Vector<2>((lx + rx) / 2.0, (ly + ry) / 2.0);
}

double TargetWidth(double lx, double ly, double rx, double ry) {
  double dx = lx - rx;
  double dy = ly - ry;
  return ::std::hypot(dx, dy);
}

void SelectTargets(const VisionData &left_target,
                   const VisionData &right_target,
                   ::aos::vision::Vector<2> *center_left,
                   ::aos::vision::Vector<2> *center_right, double *angle_left,
                   double *angle_right) {
  // No good targets. Let the caller decide defaults.
  if (right_target.target_size() == 0 || left_target.target_size() == 0) {
    return;
  }

  // Only one option, we have to go with it.
  if (right_target.target_size() == 1 && left_target.target_size() == 1) {
    *center_left =
        CreateCenterFromTarget(left_target.target(0).left_corner_x(),
                               left_target.target(0).left_corner_y(),
                               left_target.target(0).right_corner_x(),
                               left_target.target(0).right_corner_y());
    *center_right =
        CreateCenterFromTarget(right_target.target(0).left_corner_x(),
                               right_target.target(0).left_corner_y(),
                               right_target.target(0).right_corner_x(),
                               right_target.target(0).right_corner_y());
    return;
  }

  // Now we have to make a decision.
  double min_angle = -1.0;
  int left_index = 0;
  // First pick the widest target from the left.
  for (int i = 0; i < left_target.target_size(); i++) {
    const double h = left_target.target(i).left_corner_y() -
                     left_target.target(i).right_corner_y();
    const double wid1 = TargetWidth(left_target.target(i).left_corner_x(),
                                    left_target.target(i).left_corner_y(),
                                    left_target.target(i).right_corner_x(),
                                    left_target.target(i).right_corner_y());
    const double angle = h / wid1;
    if (min_angle == -1.0 || ::std::abs(angle) < ::std::abs(min_angle)) {
      min_angle = angle;
      *angle_left = angle;
      left_index = i;
    }
  }
  // Calculate the angle of the bottom edge for the left.
  double h = left_target.target(left_index).left_corner_y() -
             left_target.target(left_index).right_corner_y();

  double good_ang = min_angle;
  double min_ang_err = -1.0;
  int right_index = -1;
  // Now pick the bottom edge angle from the right that lines up best with the
  // left.
  for (int j = 0; j < right_target.target_size(); j++) {
    double wid2 = TargetWidth(right_target.target(j).left_corner_x(),
                              right_target.target(j).left_corner_y(),
                              right_target.target(j).right_corner_x(),
                              right_target.target(j).right_corner_y());
    h = right_target.target(j).left_corner_y() -
        right_target.target(j).right_corner_y();
    double ang = h / wid2;
    double ang_err = ::std::abs(good_ang - ang);
    if (min_ang_err == -1.0 || min_ang_err > ang_err) {
      min_ang_err = ang_err;
      right_index = j;
      *angle_right = ang;
    }
  }

  *center_left =
      CreateCenterFromTarget(left_target.target(left_index).left_corner_x(),
                             left_target.target(left_index).left_corner_y(),
                             left_target.target(left_index).right_corner_x(),
                             left_target.target(left_index).right_corner_y());
  *center_right =
      CreateCenterFromTarget(right_target.target(right_index).left_corner_x(),
                             right_target.target(right_index).left_corner_y(),
                             right_target.target(right_index).right_corner_x(),
                             right_target.target(right_index).right_corner_y());
}

class CameraHandler {
 public:
  void Received(const VisionData &target, monotonic_clock::time_point now) {
    if (current_.received) {
      last_ = current_;
    }
    current_.target = target;
    current_.rx_time = now;
    current_.capture_time = now -
                            chrono::nanoseconds(target.send_timestamp() -
                                                target.image_timestamp()) +
                            // It takes a bit to shoot a frame.  Push the frame
                            // further back in time.
                            chrono::milliseconds(10);
    current_.received = true;
  }

  void CheckStale(monotonic_clock::time_point now) {
    if (now > current_.rx_time + chrono::milliseconds(50)) {
      current_.received = false;
      last_.received = false;
    }
  }

  bool received_both() const { return current_.received && last_.received; }

  bool is_valid() const {
    return current_.target.target_size() > 0 && last_.target.target_size() > 0;
  }

  const VisionData &target() const { return current_.target; }
  const VisionData &last_target() const { return last_.target; }

  monotonic_clock::time_point capture_time() const {
    return current_.capture_time;
  }
  monotonic_clock::time_point last_capture_time() const {
    return last_.capture_time;
  }

 private:
  struct TargetWithTimes {
    VisionData target;
    monotonic_clock::time_point rx_time{monotonic_clock::epoch()};
    monotonic_clock::time_point capture_time{monotonic_clock::epoch()};
    bool received = false;
  };

  TargetWithTimes current_;
  TargetWithTimes last_;
};

void CalculateFiltered(const CameraHandler &older, const CameraHandler &newer,
                       const ::aos::vision::Vector<2> &newer_center,
                       const ::aos::vision::Vector<2> &last_newer_center,
                       double angle, double last_angle,
                       ::aos::vision::Vector<2> *interpolated_result,
                       double *interpolated_angle) {
  const double age_ratio =
      ::aos::time::DurationInSeconds(older.capture_time() -
                                     newer.last_capture_time()) /
      ::aos::time::DurationInSeconds(newer.capture_time() -
                                     newer.last_capture_time());
  interpolated_result->Set(
      newer_center.x() * age_ratio + (1 - age_ratio) * last_newer_center.x(),
      newer_center.y() * age_ratio + (1 - age_ratio) * last_newer_center.y());

  *interpolated_angle = angle * age_ratio + (1 - age_ratio) * last_angle;
}

// Handles calculating drivetrain offsets.
class DrivetrainOffsetCalculator {
 public:
  DrivetrainOffsetCalculator(::aos::EventLoop *event_loop)
      : drivetrain_status_fetcher_(
            event_loop
                ->MakeFetcher<::frc971::control_loops::drivetrain::Status>(
                    "/drivetrain")) {}

  // Takes a vision status message with everything except
  // drivetrain_{left,right}_position set and sets those.
  // Returns false if it doesn't have enough data to fill them out.
  bool CompleteVisionStatus(::y2016::vision::VisionStatusT *status) {
    while (drivetrain_status_fetcher_.FetchNext()) {
      data_[data_index_].time =
          drivetrain_status_fetcher_.context().monotonic_event_time;
      data_[data_index_].left =
          drivetrain_status_fetcher_->estimated_left_position();
      data_[data_index_].right =
          drivetrain_status_fetcher_->estimated_right_position();
      ++data_index_;
      if (data_index_ == data_.size()) data_index_ = 0;
      if (valid_data_ < data_.size()) ++valid_data_;
    }

    if (valid_data_ == 0) return false;

    const monotonic_clock::time_point capture_time =
        monotonic_clock::time_point(chrono::nanoseconds(status->target_time));
    DrivetrainData before, after;
    FindBeforeAfter(&before, &after, capture_time);

    if (before.time == after.time) {
      status->drivetrain_left_position = before.left;
      status->drivetrain_right_position = before.right;
    } else {
      const double age_ratio =
          ::aos::time::DurationInSeconds(capture_time - before.time) /
          ::aos::time::DurationInSeconds(after.time - before.time);
      status->drivetrain_left_position =
          before.left * (1 - age_ratio) + after.left * age_ratio;
      status->drivetrain_right_position =
          before.right * (1 - age_ratio) + after.right * age_ratio;
    }

    return true;
  }

 private:
  struct DrivetrainData {
    monotonic_clock::time_point time;
    double left, right;
  };

  // Fills out before and after with the data surrounding capture_time.
  // They might be identical if that's the closest approximation.
  // Do not call this if valid_data_ is 0.
  void FindBeforeAfter(DrivetrainData *before, DrivetrainData *after,
                       monotonic_clock::time_point capture_time) {
    size_t location = 0;
    while (true) {
      // We hit the end of our data. Just fill them both out as the last data
      // point.
      if (location >= valid_data_) {
        *before = *after =
            data_[previous_index((valid_data_ + data_index_) % data_.size())];
        return;
      }

      // The index into data_ corresponding to location positions after
      // (data_index_ - 1).
      const size_t index = previous_index(location + data_index_);

      // If we've found the one we want.
      if (data_[index].time > capture_time) {
        *after = data_[index];
        if (location == 0) {
          // If this is the first one and it's already after, just return the
          // same thing for both.
          *before = data_[index];
        } else {
          *before = data_[previous_index(index)];
        }
        return;
      }

      ++location;
    }
  }

  size_t previous_index(size_t index) const {
    if (index == 0) {
      return data_.size() - 1;
    } else {
      return index - 1;
    }
  }

  ::aos::Fetcher<::frc971::control_loops::drivetrain::Status>
      drivetrain_status_fetcher_;

  ::std::array<DrivetrainData, 200> data_;
  // The index into data_ the next data point is going at.
  size_t data_index_ = 0;
  // How many elemets of data_ are valid.
  size_t valid_data_ = 0;
};

void Main() {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("config.json");

  ::aos::ShmEventLoop event_loop(&config.message());

  ::aos::Sender<::y2016::vision::VisionStatus> vision_status_sender =
      event_loop.MakeSender<::y2016::vision::VisionStatus>("/superstructure");

  StereoGeometry stereo(constants::GetValues().vision_name);
  AOS_LOG(INFO, "calibration: %s\n",
          stereo.calibration().ShortDebugString().c_str());

  DrivetrainOffsetCalculator drivetrain_offset(&event_loop);

  CameraHandler left;
  CameraHandler right;

  ::aos::events::RXUdpSocket recv(8080);
  char rawData[65507];

  while (true) {
    // TODO(austin): Don't malloc.
    VisionData target;
    int size = recv.Recv(rawData, 65507);
    monotonic_clock::time_point now = monotonic_clock::now();

    if (target.ParseFromArray(rawData, size)) {
      if (target.camera_index() == 0) {
        left.Received(target, now);
      } else {
        right.Received(target, now);
      }
    } else {
      AOS_LOG(ERROR, "oh noes: parse error\n");
      continue;
    }

    left.CheckStale(now);
    right.CheckStale(now);

    if (left.received_both() && right.received_both()) {
      const bool left_image_valid = left.is_valid();
      const bool right_image_valid = right.is_valid();

      auto builder = vision_status_sender.MakeBuilder();
      VisionStatusT new_vision_status;
      new_vision_status.left_image_valid = left_image_valid;
      new_vision_status.right_image_valid = right_image_valid;
      if (left_image_valid && right_image_valid) {
        ::aos::vision::Vector<2> center_left(0.0, 0.0);
        ::aos::vision::Vector<2> center_right(0.0, 0.0);
        double angle_left;
        double angle_right;
        SelectTargets(left.target(), right.target(), &center_left,
                      &center_right, &angle_left, &angle_right);

        // TODO(Ben): Remember this from last time instead of recalculating it
        // each time.
        ::aos::vision::Vector<2> last_center_left(0.0, 0.0);
        ::aos::vision::Vector<2> last_center_right(0.0, 0.0);
        double last_angle_left;
        double last_angle_right;
        SelectTargets(left.last_target(), right.last_target(),
                      &last_center_left, &last_center_right, &last_angle_left,
                      &last_angle_right);

        ::aos::vision::Vector<2> filtered_center_left(0.0, 0.0);
        ::aos::vision::Vector<2> filtered_center_right(0.0, 0.0);
        double filtered_angle_left;
        double filtered_angle_right;
        if (left.capture_time() < right.capture_time()) {
          filtered_center_left = center_left;
          filtered_angle_left = angle_left;
          new_vision_status.target_time =
              chrono::duration_cast<chrono::nanoseconds>(
                  left.capture_time().time_since_epoch())
                  .count();
          CalculateFiltered(left, right, center_right, last_center_right,
                            angle_right, last_angle_right,
                            &filtered_center_right, &filtered_angle_right);
        } else {
          filtered_center_right = center_right;
          filtered_angle_right = angle_right;
          new_vision_status.target_time =
              chrono::duration_cast<chrono::nanoseconds>(
                  right.capture_time().time_since_epoch())
                  .count();
          CalculateFiltered(right, left, center_left, last_center_left,
                            angle_left, last_angle_left, &filtered_center_left,
                            &filtered_angle_left);
        }

        double distance, horizontal_angle, vertical_angle;
        stereo.Process(filtered_center_left, filtered_center_right, &distance,
                       &horizontal_angle, &vertical_angle);
        new_vision_status.left_image_timestamp =
            left.target().image_timestamp();
        new_vision_status.right_image_timestamp =
            right.target().image_timestamp();
        new_vision_status.left_send_timestamp = left.target().send_timestamp();
        new_vision_status.right_send_timestamp =
            right.target().send_timestamp();
        new_vision_status.horizontal_angle = horizontal_angle;
        new_vision_status.vertical_angle = vertical_angle;
        new_vision_status.distance = distance;
        new_vision_status.angle =
            (filtered_angle_left + filtered_angle_right) / 2.0;
      }

      if (drivetrain_offset.CompleteVisionStatus(&new_vision_status)) {
        if (!builder.Send(
                VisionStatus::Pack(*builder.fbb(), &new_vision_status))) {
          AOS_LOG(ERROR, "Failed to send vision information\n");
        }
      } else {
        AOS_LOG(WARNING, "vision without drivetrain");
      }
    }

    if (target.camera_index() == 0) {
      AOS_LOG(DEBUG, "left_target: %s\n",
              left.target().ShortDebugString().c_str());
    } else {
      AOS_LOG(DEBUG, "right_target: %s\n",
              right.target().ShortDebugString().c_str());
    }
  }
}

}  // namespace vision
}  // namespace y2016

int main(int /*argc*/, char ** /*argv*/) {
  ::aos::InitNRT();
  ::y2016::vision::Main();
}
