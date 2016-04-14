#include <stdlib.h>
#include <netdb.h>
#include <unistd.h>

#include <vector>
#include <memory>

#include "aos/linux_code/init.h"
#include "aos/common/time.h"
#include "aos/common/logging/logging.h"
#include "aos/common/logging/queue_logging.h"
#include "aos/vision/events/udp.h"

#include "y2016/vision/vision.q.h"
#include "y2016/vision/vision_data.pb.h"
#include "y2016/vision/stereo_geometry.h"
#include "y2016/constants.h"

namespace y2016 {
namespace vision {

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
                   ::aos::vision::Vector<2> *center_right) {
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
  void Received(const VisionData &target, ::aos::time::Time now) {
    if (current_.received) {
      last_ = current_;
    }
    current_.target = target;
    current_.rx_time = now;
    current_.capture_time =
        now - ::aos::time::Time::InNS(target.send_timestamp() -
                                      target.image_timestamp());
    current_.received = true;
  }

  void CheckStale(::aos::time::Time now) {
    if (now > current_.rx_time + ::aos::time::Time::InMS(50)) {
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

  ::aos::time::Time capture_time() const { return current_.capture_time; }
  ::aos::time::Time last_capture_time() const { return last_.capture_time; }

 private:
  struct TargetWithTimes {
    VisionData target;
    ::aos::time::Time rx_time{0, 0};
    ::aos::time::Time capture_time{0, 0};
    bool received = false;
  };

  TargetWithTimes current_;
  TargetWithTimes last_;
};

::aos::vision::Vector<2> CalculateFiltered(
    const CameraHandler &older, const CameraHandler &newer,
    const ::aos::vision::Vector<2> &newer_center,
    const ::aos::vision::Vector<2> &last_newer_center) {
  const double age_ratio =
      (older.capture_time() - newer.last_capture_time()).ToSeconds() /
      (newer.capture_time() - newer.last_capture_time()).ToSeconds();
  return ::aos::vision::Vector<2>(
      newer_center.x() * age_ratio + (1 - age_ratio) * last_newer_center.x(),
      newer_center.y() * age_ratio + (1 - age_ratio) * last_newer_center.y());
}

void Main() {
  StereoGeometry stereo(constants::GetValues().vision_name);
  LOG(INFO, "calibration: %s\n",
      stereo.calibration().ShortDebugString().c_str());

  CameraHandler left;
  CameraHandler right;

  ::aos::vision::RXUdpSocket recv(8080);
  char rawData[65507];

  while (true) {
    // TODO(austin): Don't malloc.
    VisionData target;
    int size = recv.Recv(rawData, 65507);
    ::aos::time::Time now = ::aos::time::Time::Now();

    if (target.ParseFromArray(rawData, size)) {
      if (target.camera_index() == 0) {
        left.Received(target, now);
      } else {
        right.Received(target, now);
      }
    } else {
      LOG(ERROR, "oh noes: parse error\n");
      continue;
    }

    left.CheckStale(now);
    right.CheckStale(now);

    if (left.received_both() && right.received_both()) {
      const bool left_image_valid = left.is_valid();
      const bool right_image_valid = right.is_valid();

      auto new_vision_status = vision_status.MakeMessage();
      new_vision_status->left_image_valid = left_image_valid;
      new_vision_status->right_image_valid = right_image_valid;
      if (left_image_valid && right_image_valid) {
        ::aos::vision::Vector<2> center_left(0.0, 0.0);
        ::aos::vision::Vector<2> center_right(0.0, 0.0);
        SelectTargets(left.target(), right.target(), &center_left,
                      &center_right);

        // TODO(Ben): Remember this from last time instead of recalculating it
        // each time.
        ::aos::vision::Vector<2> last_center_left(0.0, 0.0);
        ::aos::vision::Vector<2> last_center_right(0.0, 0.0);
        SelectTargets(left.last_target(), right.last_target(),
                      &last_center_left, &last_center_right);

        ::aos::vision::Vector<2> filtered_center_left(0.0, 0.0);
        ::aos::vision::Vector<2> filtered_center_right(0.0, 0.0);
        if (left.capture_time() < right.capture_time()) {
          filtered_center_left = center_left;
          new_vision_status->target_time = left.capture_time().ToNSec();
          filtered_center_right =
              CalculateFiltered(left, right, center_right, last_center_right);
        } else {
          filtered_center_right = center_right;
          new_vision_status->target_time = right.capture_time().ToNSec();
          filtered_center_right =
              CalculateFiltered(right, left, center_left, last_center_left);
        }

        double distance, horizontal_angle, vertical_angle;
        stereo.Process(filtered_center_left, filtered_center_right, &distance,
                       &horizontal_angle, &vertical_angle);
        new_vision_status->left_image_timestamp =
            left.target().image_timestamp();
        new_vision_status->right_image_timestamp =
            right.target().image_timestamp();
        new_vision_status->left_send_timestamp = left.target().send_timestamp();
        new_vision_status->right_send_timestamp = right.target().send_timestamp();
        new_vision_status->horizontal_angle = horizontal_angle;
        new_vision_status->vertical_angle = vertical_angle;
        new_vision_status->distance = distance;
      }
      LOG_STRUCT(DEBUG, "vision", *new_vision_status);

      if (!new_vision_status.Send()) {
        LOG(ERROR, "Failed to send vision information\n");
      }
    }

    if (target.camera_index() == 0) {
      LOG(DEBUG, "left_target: %s\n", left.target().ShortDebugString().c_str());
    } else {
      LOG(DEBUG, "right_target: %s\n",
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
