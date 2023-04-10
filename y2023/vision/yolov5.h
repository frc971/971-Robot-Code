#ifndef Y2023_VISION_YOLOV5_H_
#define Y2023_VISION_YOLOV5_H_

#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

namespace y2023 {
namespace vision {

struct Detection {
  cv::Rect box;
  double confidence;
  int class_id;
};

class YOLOV5 {
 public:
  virtual ~YOLOV5() {}

  // Takes a model path as string and loads a pre-trained
  // YOLOv5 model from the specified path.
  virtual void LoadModel(const std::string path) = 0;

  // Takes an image and returns a Detection.
  virtual std::vector<Detection> ProcessImage(cv::Mat image) = 0;
};

std::unique_ptr<YOLOV5> MakeYOLOV5();

}  // namespace vision
}  // namespace y2023

#endif  // Y2023_VISION_YOLOV5_H_
