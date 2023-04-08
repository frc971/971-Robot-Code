#ifndef Y2023_VISION_YOLOV5_H_
#define Y2023_VISION_YOLOV5_H_

#include <tensorflow/lite/interpreter.h>
#include <tensorflow/lite/kernels/register.h>
#include <tensorflow/lite/model.h>
#include <tflite/public/edgetpu_c.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
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
  // Takes a model path as string and loads a pre-trained
  // YOLOv5 model from the specified path.
  void LoadModel(const std::string path);

  // Takes an image and returns a Detection.
  std::vector<Detection> ProcessImage(cv::Mat image);

 private:
  // Convert an OpenCV Mat object to a tensor input
  // that can be fed to the TensorFlow Lite model.
  void ConvertCVMatToTensor(const cv::Mat &src, uint8_t *in);

  // Resizes, converts color space, and converts
  // image data type before inference.
  void Preprocess(cv::Mat image);

  // Converts a TensorFlow Lite tensor to a 2D vector.
  std::vector<std::vector<float>> TensorToVector2D(TfLiteTensor *src_tensor,
                                                   const int rows,
                                                   const int columns);

  // Performs non-maximum suppression to remove overlapping bounding boxes.
  void NonMaximumSupression(const std::vector<std::vector<float>> &orig_preds,
                            const int rows, const int columns,
                            std::vector<Detection> *detections,
                            std::vector<int> *indices);
  // Models
  std::unique_ptr<tflite::FlatBufferModel> model_;
  std::unique_ptr<tflite::Interpreter> interpreter_;
  tflite::StderrReporter error_reporter_;

  // Parameters of interpreter's input
  int input_;
  int in_height_;
  int in_width_;
  int in_channels_;
  int in_type_;

  // Parameters of original image
  int img_height_;
  int img_width_;

  // Input of the interpreter
  uint8_t *input_8_;

  // Subtract this offset from class labels to get the actual label.
  static constexpr int kClassIdOffset = 5;
};

}  // namespace vision
}  // namespace y2023

#endif  // Y2023_VISION_YOLOV5_H_
