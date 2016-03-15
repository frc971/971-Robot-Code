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
  VisionData right_target;
  // TODO(Brian): Having two sockets here like this doesn't work. They'll get
  // out of sync.
  ::aos::vision::RXUdpSocket left(8080);
  ::aos::vision::RXUdpSocket right(8081);
  char rawData[65507];
  while (true) {
    bool got_left = false;
    bool got_right = false;
    int recv = left.Recv(rawData, 65507);
    if (left_target.ParseFromArray(rawData, recv)) {
      LOG(DEBUG, "left_target: %s\n", left_target.ShortDebugString().c_str());
      if (left_target.target_size() > 0) {
        got_left = true;
      }
    } else {
      LOG(WARNING, "left parse error\n");
    }

    recv = right.Recv(rawData, 65507);
    if (right_target.ParseFromArray(rawData, recv)) {
      LOG(DEBUG, "right_target: %s\n", right_target.ShortDebugString().c_str());
      if (right_target.target_size() > 0) {
        got_right = true;
      }
    } else {
      LOG(WARNING, "right parse error\n");
    }

    if (got_left && got_right) {
      const ::aos::vision::Vector<2> center0(
          (left_target.target(0).left_corner_x() +
           left_target.target(0).right_corner_x()) /
              2.0,
          (left_target.target(0).left_corner_y() +
           left_target.target(0).right_corner_y()) /
              2.0);
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
      auto new_vision_status = vision_status.MakeMessage();
      new_vision_status->left_image_timestamp = left_target.image_timestamp();
      new_vision_status->right_image_timestamp = right_target.image_timestamp();
      new_vision_status->left_send_timestamp = left_target.send_timestamp();
      new_vision_status->right_send_timestamp = right_target.send_timestamp();
      new_vision_status->horizontal_angle = horizontal_angle;
      new_vision_status->vertical_angle = vertical_angle;
      new_vision_status->distance = distance;
      LOG_STRUCT(DEBUG, "vision", *new_vision_status);

      if (!new_vision_status.Send()) {
        LOG(ERROR, "Failed to send vision information\n");
      }
    }
  }
}

}  // namespace vision
}  // namespace y2016

int main(int /*argc*/, char ** /*argv*/) {
  ::aos::InitNRT();
  ::y2016::vision::Main();
}
