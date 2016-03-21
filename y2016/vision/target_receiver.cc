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

::aos::vision::Vector<2> CreateCenterFromTarget(double lx, double ly, double rx, double ry) {
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
  // Now pick the bottom edge angle from the right that lines up best with the left.
  for (int j = 0; j < right_target.target_size(); j++) {
    double wid2 = TargetWidth(right_target.target(j).left_corner_x(),
                                right_target.target(j).left_corner_y(),
                                right_target.target(j).right_corner_x(),
                                right_target.target(j).right_corner_y());
    h = right_target.target(j).left_corner_y() -
        right_target.target(j).right_corner_y();
    double ang = h/ wid2;
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


void Main() {
  StereoGeometry stereo(constants::GetValues().vision_name);
  LOG(INFO, "calibration: %s\n",
      stereo.calibration().ShortDebugString().c_str());

  VisionData left_target;
  aos::time::Time left_rx_time{0, 0};

  VisionData right_target;
  aos::time::Time right_rx_time{0, 0};

  ::aos::vision::RXUdpSocket recv(8080);
  char rawData[65507];
  bool got_left = false;
  bool got_right = false;

  while (true) {
    // TODO(austin): Don't malloc.
    VisionData target;
    int size = recv.Recv(rawData, 65507);
    aos::time::Time now = aos::time::Time::Now();

    if (target.ParseFromArray(rawData, size)) {
      if (target.camera_index() == 0) {
        left_target = target;
        left_rx_time = now;
        got_left = true;
      } else {
        right_target = target;
        right_rx_time = now;
        got_right = true;
      }
    } else {
      LOG(ERROR, "oh noes: parse error\n");
      continue;
    }

    if (now > left_rx_time + aos::time::Time::InMS(50)) {
      got_left = false;
    }
    if (now > right_rx_time + aos::time::Time::InMS(50)) {
      got_right = false;
    }

    if (got_left && got_right) {
      bool left_image_valid = left_target.target_size() > 0;
      bool right_image_valid = right_target.target_size() > 0;

      auto new_vision_status = vision_status.MakeMessage();
      new_vision_status->left_image_valid = left_image_valid;
      new_vision_status->right_image_valid = right_image_valid;
      if (left_image_valid && right_image_valid) {
        ::aos::vision::Vector<2> center0(0.0, 0.0);
        ::aos::vision::Vector<2> center1(0.0, 0.0);
        SelectTargets(left_target, right_target, &center0, &center1);
        double distance, horizontal_angle, vertical_angle;
        stereo.Process(center0, center1, &distance, &horizontal_angle,
                       &vertical_angle);
        new_vision_status->left_image_timestamp = left_target.image_timestamp();
        new_vision_status->right_image_timestamp = right_target.image_timestamp();
        new_vision_status->left_send_timestamp = left_target.send_timestamp();
        new_vision_status->right_send_timestamp = right_target.send_timestamp();
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
      LOG(DEBUG, "left_target: %s\n", left_target.ShortDebugString().c_str());
    } else {
      LOG(DEBUG, "right_target: %s\n", right_target.ShortDebugString().c_str());
    }
  }
}

}  // namespace vision
}  // namespace y2016

int main(int /*argc*/, char ** /*argv*/) {
  ::aos::InitNRT();
  ::y2016::vision::Main();
}
