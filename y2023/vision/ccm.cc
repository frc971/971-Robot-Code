#include "opencv2/mcc/ccm.hpp"

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/mcc.hpp"

#include "aos/init.h"
#include "aos/time/time.h"

namespace y2023::vision {
namespace {

// Adapted from the opencv example code for color calibration.
void CCMMain(std::string filepath) {
  cv::Mat image = cv::imread(filepath, cv::IMREAD_COLOR);

  CHECK(image.data) << "Failed to read " << filepath;

  cv::Mat imageCopy = image.clone();
  cv::Ptr<cv::mcc::CCheckerDetector> detector =
      cv::mcc::CCheckerDetector::create();

  // Marker type to detect
  CHECK(detector->process(image, cv::mcc::MCC24, /*max number of charts*/ 1))
      << ": Failed to detect chart.";

  std::vector<cv::Ptr<cv::mcc::CChecker>> checkers =
      detector->getListColorChecker();
  for (cv::Ptr<cv::mcc::CChecker> checker : checkers) {
    cv::Ptr<cv::mcc::CCheckerDraw> cdraw =
        cv::mcc::CCheckerDraw::create(checker);
    cdraw->draw(image);
    cv::Mat charts_rgb = checker->getChartsRGB();
    cv::Mat src = charts_rgb.col(1).clone().reshape(3, charts_rgb.rows / 3);
    src /= 255.0;

    // compte color correction matrix
    cv::ccm::ColorCorrectionModel model1(src, cv::ccm::COLORCHECKER_Macbeth);
    model1.run();
    cv::Mat ccm = model1.getCCM();
    LOG(INFO) << "ccm " << ccm;
    for (int i = 0; i < 9; ++i) {
      LOG(INFO) << "mat[" << i << "] = " << ccm.at<double>(i) << ";";
    }
    double loss = model1.getLoss();
    LOG(INFO) << "loss " << loss;

    cv::Mat img_;
    cv::cvtColor(image, img_, cv::COLOR_BGR2RGB);
    img_.convertTo(img_, CV_64F);
    const int inp_size = 255;
    const int out_size = 255;
    img_ = img_ / inp_size;
    cv::Mat calibrated_image = model1.infer(img_);
    cv::Mat out_ = calibrated_image * out_size;

    // Save the calibrated image to {FILE_NAME}.calibrated.{FILE_EXT}
    out_.convertTo(out_, CV_8UC3);
    cv::Mat img_out = min(max(out_, 0), out_size);
    cv::Mat out_img;
    cv::cvtColor(img_out, out_img, cv::COLOR_RGB2BGR);

    std::string filename = filepath.substr(filepath.find_last_of('/') + 1);
    size_t dot_index = filename.find_last_of('.');
    std::string base_name = filename.substr(0, dot_index);
    std::string ext =
        filename.substr(dot_index + 1, filename.length() - dot_index);
    std::string calibrated_file_path = base_name + ".calibrated." + ext;
    cv::imwrite(calibrated_file_path, out_img);
  }
}

}  // namespace
}  // namespace y2023::vision

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  if (argc != 2) {
    LOG(FATAL) << "Expected filename as an argument.";
  }
  y2023::vision::CCMMain(argv[1]);
}
