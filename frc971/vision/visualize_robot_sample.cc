#include "frc971/vision/visualize_robot.h"

#include "aos/init.h"
#include "aos/logging/logging.h"
#include "glog/logging.h"

#include "Eigen/Dense"

#include <math.h>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "aos/time/time.h"

namespace frc971 {
namespace vision {

// Show / test the basics of visualizing the robot frames
void Main(int /*argc*/, char ** /* argv */) {
  VisualizeRobot vis_robot;

  int image_width = 500;
  cv::Mat image_mat =
      cv::Mat::zeros(cv::Size(image_width, image_width), CV_8UC3);
  vis_robot.SetImage(image_mat);

  // 10 meters above the origin, rotated so the camera faces straight down
  Eigen::Translation3d camera_trans(0, 0, 10.0);
  Eigen::AngleAxisd camera_rot(M_PI, Eigen::Vector3d::UnitX());
  Eigen::Affine3d camera_viewpoint = camera_trans * camera_rot;
  vis_robot.SetViewpoint(camera_viewpoint);

  cv::Mat camera_mat;
  double focal_length = 1000.0;
  double intr[] = {focal_length, 0.0,          image_width / 2.0,
                   0.0,          focal_length, image_width / 2.0,
                   0.0,          0.0,          1.0};
  camera_mat = cv::Mat(3, 3, CV_64FC1, intr);
  vis_robot.SetCameraParameters(camera_mat);

  Eigen::Affine3d offset_rotate_origin(Eigen::Affine3d::Identity());

  cv::Mat dist_coeffs = cv::Mat(1, 5, CV_64F, 0.0);
  vis_robot.SetDistortionCoefficients(dist_coeffs);

  // Go around the clock and plot the coordinate frame at different rotations
  for (int i = 0; i < 12; i++) {
    double angle = M_PI * double(i) / 6.0;
    Eigen::Vector3d trans;
    trans << 1.0 * cos(angle), 1.0 * sin(angle), 0.0;

    offset_rotate_origin = Eigen::Translation3d(trans) *
                           Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX());

    vis_robot.DrawFrameAxes(offset_rotate_origin, std::to_string(i));
  }

  // Display the result
  cv::imshow("Display", image_mat);
  cv::waitKey();
}
}  // namespace vision
}  // namespace frc971

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  frc971::vision::Main(argc, argv);
}
