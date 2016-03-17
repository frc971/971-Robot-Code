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
        const ::aos::vision::Vector<2> center0(
            (left_target.target(0).left_corner_x() +
             left_target.target(0).right_corner_x()) /
                2.0,
            (left_target.target(0).left_corner_y() +
             left_target.target(0).right_corner_y()) /
                2.0);
        // out of sync.
        const ::aos::vision::Vector<2> center1(
            (right_target.target(0).left_corner_x() +
             right_target.target(0).right_corner_x()) /
                2.0,
            (right_target.target(0).left_corner_y() +
             right_target.target(0).right_corner_y()) /
                2.0);
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
