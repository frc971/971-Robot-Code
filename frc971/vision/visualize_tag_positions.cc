#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "opencv2/opencv.hpp"

#include "aos/json_to_flatbuffer.h"
#include "frc971/vision/target_map_field_generated.h"
#include "frc971/vision/target_map_utils.h"

ABSL_FLAG(std::string, path_to_positions, "y2025/vision/maps/target_map.json",
          "path to the target map (must be added to BUILD)");
ABSL_FLAG(int, field_width, 512, "the width of the field display, in pixels");
ABSL_FLAG(int, field_length, 1024,
          "the length of the field display, in pixels");
ABSL_FLAG(int, field_height, 512, "the height of the field display, in pixels");
ABSL_FLAG(double, pixels_per_meter, 50.0,
          "the number of pixels which correspond to a meter");
ABSL_FLAG(double, axis_length, 25.0, "the length in pixels of the axes");

int main(int argc, char **argv) {
  absl::ParseCommandLine(argc, argv);

  const double pixels_per_meter = absl::GetFlag(FLAGS_pixels_per_meter);
  const double field_width = absl::GetFlag(FLAGS_field_width);
  const double field_length = absl::GetFlag(FLAGS_field_length);
  const double field_height = absl::GetFlag(FLAGS_field_height);
  const double axis_length = absl::GetFlag(FLAGS_axis_length);

  cv::Mat topImage =
      cv::Mat::zeros(cv::Size(field_length, field_width), CV_8UC3);
  cv::Mat frontImage =
      cv::Mat::zeros(cv::Size(field_width, field_height), CV_8UC3);
  cv::Mat sideImage =
      cv::Mat::zeros(cv::Size(field_length, field_height), CV_8UC3);

  const frc971::vision::TargetMapField &target_map =
      aos::JsonFileToFlatbuffer<frc971::vision::TargetMapField>(
          absl::GetFlag(FLAGS_path_to_positions))
          .message();

  std::unordered_map<int, Eigen::Matrix<double, 4, 4>> transforms;
  Eigen::Matrix<double, 4, 4> convert_to_screen =
      Eigen::Matrix<double, 4, 4>::Identity();
  convert_to_screen(2, 2) = -1.0;
  convert_to_screen(1, 1) = -1.0;
  convert_to_screen *= pixels_per_meter;
  convert_to_screen(0, 3) = field_length / 2;
  convert_to_screen(1, 3) = field_width / 2;
  convert_to_screen(2, 3) = field_height - axis_length * 2;
  for (size_t i = 0; i < target_map.target_poses()->size(); i++) {
    transforms.emplace(
        target_map.target_poses()->Get(i)->id(),
        convert_to_screen *
            frc971::vision::PoseToTransform(target_map.target_poses()->Get(i)));
  }

  for (std::pair<int, Eigen::Matrix<double, 4, 4>> transform : transforms) {
    Eigen::Matrix<double, 3, 1> x_axis =
        transform.second.block<3, 1>(0, 0).eval() / pixels_per_meter *
        axis_length;
    Eigen::Matrix<double, 3, 1> y_axis =
        transform.second.block<3, 1>(0, 1).eval() / pixels_per_meter *
        axis_length;
    Eigen::Matrix<double, 3, 1> z_axis =
        transform.second.block<3, 1>(0, 2).eval() / pixels_per_meter *
        axis_length;
    Eigen::Matrix<double, 3, 1> origin =
        transform.second.block<3, 1>(0, 3).eval();

    cv::Point topOrigin((int)origin(0), (int)origin(1));
    cv::Point frontOrigin((int)origin(1), (int)origin(2));
    cv::Point sideOrigin((int)origin(0), (int)origin(2));

    cv::line(topImage, topOrigin,
             topOrigin + cv::Point((int)x_axis(0), (int)x_axis(1)),
             cv::Scalar(0, 0, 255), 2);
    cv::line(topImage, topOrigin,
             topOrigin + cv::Point((int)z_axis(0), (int)z_axis(1)),
             cv::Scalar(255, 0, 0), 2);
    cv::line(topImage, topOrigin,
             topOrigin + cv::Point((int)y_axis(0), (int)y_axis(1)),
             cv::Scalar(0, 255, 0), 2);
    cv::putText(topImage, std::to_string(transform.first), topOrigin,
                cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255, 255, 255), 1);

    cv::line(frontImage, frontOrigin,
             frontOrigin + cv::Point((int)x_axis(1), (int)x_axis(2)),
             cv::Scalar(0, 0, 255), 2);
    cv::line(frontImage, frontOrigin,
             frontOrigin + cv::Point((int)z_axis(1), (int)z_axis(2)),
             cv::Scalar(255, 0, 0), 2);
    cv::line(frontImage, frontOrigin,
             frontOrigin + cv::Point((int)y_axis(1), (int)y_axis(2)),
             cv::Scalar(0, 255, 0), 2);
    cv::putText(frontImage, std::to_string(transform.first), frontOrigin,
                cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255, 255, 255), 1);

    cv::line(sideImage, sideOrigin,
             sideOrigin + cv::Point((int)x_axis(0), (int)x_axis(2)),
             cv::Scalar(0, 0, 255), 2);
    cv::line(sideImage, sideOrigin,
             sideOrigin + cv::Point((int)z_axis(0), (int)z_axis(2)),
             cv::Scalar(255, 0, 0), 2);
    cv::line(sideImage, sideOrigin,
             sideOrigin + cv::Point((int)y_axis(0), (int)y_axis(2)),
             cv::Scalar(0, 255, 0), 2);
    cv::putText(sideImage, std::to_string(transform.first), sideOrigin,
                cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255, 255, 255), 1);
  }

  cv::imshow("AprilTag visualization (top)", topImage);
  cv::imshow("AprilTag visualization (front)", frontImage);
  cv::imshow("AprilTag visualization (side)", sideImage);
  cv::waitKey(0);

  return 0;
}