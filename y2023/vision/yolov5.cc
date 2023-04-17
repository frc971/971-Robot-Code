#include "yolov5.h"

#include <tensorflow/lite/c/common.h>
#include <tensorflow/lite/interpreter.h>
#include <tensorflow/lite/kernels/register.h>
#include <tensorflow/lite/model.h>
#include <tflite/public/edgetpu.h>
#include <tflite/public/edgetpu_c.h>

#include <chrono>
#include <opencv2/dnn.hpp>
#include <string>

#include "absl/types/span.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

DEFINE_double(conf_threshold, 0.9,
              "Threshold value for confidence scores. Detections with a "
              "confidence score below this value will be ignored.");

DEFINE_double(
    nms_threshold, 0.5,
    "Threshold value for non-maximum suppression. Detections with an "
    "intersection-over-union value below this value will be removed.");

DEFINE_int32(nthreads, 6, "Number of threads to use during inference.");

DEFINE_bool(visualize_detections, false, "Display inference output");

namespace y2023 {
namespace vision {

class YOLOV5Impl : public YOLOV5 {
 public:
  // Takes a model path as string and and loads a pre-trained
  // YOLOv5 model from the specified path.
  void LoadModel(const std::string path);

  // Takes an image and returns a Detection.
  std::vector<Detection> ProcessImage(cv::Mat image);

 private:
  // Convert an OpenCV Mat object to a tensor input
  // that can be fed to the TensorFlow Lite model.
  void ConvertCVMatToTensor(cv::Mat src, absl::Span<uint8_t> tensor);

  // Resizes, converts color space, and converts
  // image data type before inference.
  void Preprocess(cv::Mat image);

  // Converts a TensorFlow Lite tensor to a 2D vector.
  std::vector<std::vector<float>> TensorToVector2D(TfLiteTensor *src_tensor,
                                                   const int rows,
                                                   const int columns);

  // Performs non-maximum suppression to remove overlapping bounding boxes.
  std::vector<Detection> NonMaximumSupression(
      const std::vector<std::vector<float>> &orig_preds, const int rows,
      const int columns, std::vector<Detection> *detections,
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
  absl::Span<uint8_t> input_8_;

  // Subtract this offset from class labels to get the actual label.
  static constexpr int kClassIdOffset = 5;
};

std::unique_ptr<YOLOV5> MakeYOLOV5() { return std::make_unique<YOLOV5Impl>(); }

void YOLOV5Impl::LoadModel(const std::string path) {
  VLOG(1) << "Load model: Start";

  tflite::ops::builtin::BuiltinOpResolver resolver;

  model_ = tflite::FlatBufferModel::VerifyAndBuildFromFile(path.c_str());
  CHECK(model_);
  CHECK(model_->initialized());
  VLOG(1) << "Load model: Build model from file success";

  CHECK_EQ(tflite::InterpreterBuilder(*model_, resolver)(&interpreter_),
           kTfLiteOk);
  VLOG(1) << "Load model: Interpreter builder success";

  size_t num_devices;
  std::unique_ptr<edgetpu_device, decltype(&edgetpu_free_devices)> devices(
      edgetpu_list_devices(&num_devices), &edgetpu_free_devices);

  CHECK_EQ(num_devices, 1ul);
  const auto &device = devices.get()[0];
  VLOG(1) << "Load model: Got Devices";

  auto *delegate =
      edgetpu_create_delegate(device.type, device.path, nullptr, 0);

  interpreter_->ModifyGraphWithDelegate(delegate);

  VLOG(1) << "Load model: Modify graph with delegate complete";

  TfLiteStatus status = interpreter_->AllocateTensors();
  CHECK_EQ(status, kTfLiteOk);
  CHECK(interpreter_);

  VLOG(1) << "Load model: Allocate tensors success";

  input_ = interpreter_->inputs()[0];
  TfLiteIntArray *dims = interpreter_->tensor(input_)->dims;
  in_height_ = dims->data[1];
  in_width_ = dims->data[2];
  in_channels_ = dims->data[3];
  in_type_ = interpreter_->tensor(input_)->type;

  int tensor_size = 1;
  for (int i = 0; i < dims->size; i++) {
    tensor_size *= dims->data[i];
  }
  input_8_ =
      absl::Span(interpreter_->typed_tensor<uint8_t>(input_), tensor_size);

  interpreter_->SetNumThreads(FLAGS_nthreads);

  VLOG(1) << "Load model: Done";
}

void YOLOV5Impl::ConvertCVMatToTensor(cv::Mat src, absl::Span<uint8_t> tensor) {
  CHECK(src.type() == CV_8UC3);
  int n = 0, nc = src.channels(), ne = src.elemSize();
  VLOG(2) << "ConvertCVMatToTensor: Rows " << src.rows;
  VLOG(2) << "ConvertCVMatToTensor: Cols " << src.cols;
  for (int y = 0; y < src.rows; ++y) {
    auto *row_ptr = src.ptr<uint8_t>(y);
    for (int x = 0; x < src.cols; ++x) {
      for (int c = 0; c < nc; ++c) {
        tensor[n++] = *(row_ptr + x * ne + c);
      }
    }
  }
}

std::vector<std::vector<float>> YOLOV5Impl::TensorToVector2D(
    TfLiteTensor *src_tensor, const int rows, const int columns) {
  auto scale = src_tensor->params.scale;
  auto zero_point = src_tensor->params.zero_point;
  std::vector<std::vector<float>> result_vec;
  for (int32_t i = 0; i < rows; i++) {
    std::vector<float> row_values;
    for (int32_t j = 0; j < columns; j++) {
      float val_float =
          ((static_cast<int32_t>(src_tensor->data.uint8[i * columns + j])) -
           zero_point) *
          scale;
      row_values.push_back(val_float);
    }
    result_vec.push_back(row_values);
  }
  return result_vec;
}

std::vector<Detection> YOLOV5Impl::NonMaximumSupression(
    const std::vector<std::vector<float>> &orig_preds, const int rows,
    const int columns, std::vector<Detection> *detections,
    std::vector<int> *indices)

{
  std::vector<float> scores;
  double confidence;
  cv::Point class_id;

  for (int i = 0; i < rows; i++) {
    if (orig_preds[i][4] > FLAGS_conf_threshold) {
      float x = orig_preds[i][0];
      float y = orig_preds[i][1];
      float w = orig_preds[i][2];
      float h = orig_preds[i][3];
      int left = static_cast<int>((x - 0.5 * w) * img_width_);
      int top = static_cast<int>((y - 0.5 * h) * img_height_);
      int width = static_cast<int>(w * img_width_);
      int height = static_cast<int>(h * img_height_);

      for (int j = 5; j < columns; j++) {
        scores.push_back(orig_preds[i][j] * orig_preds[i][4]);
      }

      cv::minMaxLoc(scores, nullptr, &confidence, nullptr, &class_id);
      scores.clear();
      if (confidence > FLAGS_conf_threshold) {
        Detection detection{cv::Rect(left, top, width, height), confidence,
                            class_id.x};
        detections->push_back(detection);
      }
    }
  }

  std::vector<cv::Rect> boxes;
  std::vector<float> confidences;

  for (const Detection &d : *detections) {
    boxes.push_back(d.box);
    confidences.push_back(d.confidence);
  }

  cv::dnn::NMSBoxes(boxes, confidences, FLAGS_conf_threshold,
                    FLAGS_nms_threshold, *indices);

  std::vector<Detection> filtered_detections;
  for (size_t i = 0; i < indices->size(); i++) {
    filtered_detections.push_back((*detections)[(*indices)[i]]);
  }

  VLOG(1) << "NonMaximumSupression: " << detections->size() - indices->size()
          << " detections filtered out";

  return filtered_detections;
}

std::vector<Detection> YOLOV5Impl::ProcessImage(cv::Mat frame) {
  VLOG(1) << "\n";

  auto start = std::chrono::high_resolution_clock::now();
  img_height_ = frame.rows;
  img_width_ = frame.cols;

  cv::resize(frame, frame, cv::Size(in_height_, in_width_), cv::INTER_CUBIC);
  cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
  frame.convertTo(frame, CV_8U);

  ConvertCVMatToTensor(frame, input_8_);

  TfLiteStatus status = interpreter_->Invoke();
  CHECK_EQ(status, kTfLiteOk);

  int output_tensor_index = interpreter_->outputs()[0];
  TfLiteIntArray *out_dims = interpreter_->tensor(output_tensor_index)->dims;
  int num_rows = out_dims->data[1];
  int num_columns = out_dims->data[2];

  TfLiteTensor *src_tensor = interpreter_->tensor(interpreter_->outputs()[0]);

  std::vector<std::vector<float>> orig_preds =
      TensorToVector2D(src_tensor, num_rows, num_columns);

  std::vector<int> indices;
  std::vector<Detection> detections;

  std::vector<Detection> filtered_detections;
  filtered_detections = NonMaximumSupression(orig_preds, num_rows, num_columns,
                                             &detections, &indices);
  VLOG(1) << "---";
  for (size_t i = 0; i < filtered_detections.size(); i++) {
    VLOG(1) << "Detection #" << i << " | Class ID #"
            << filtered_detections[i].class_id << "  @ "
            << filtered_detections[i].confidence << " confidence";
  }

  VLOG(1) << "---";

  auto stop = std::chrono::high_resolution_clock::now();

  VLOG(1) << "Inference time: "
          << std::chrono::duration_cast<std::chrono::milliseconds>(stop - start)
                 .count();

  if (FLAGS_visualize_detections) {
    cv::resize(frame, frame, cv::Size(img_width_, img_height_), 0, 0, true);
    for (size_t i = 0; i < filtered_detections.size(); i++) {
      VLOG(1) << "Bounding Box | X: " << filtered_detections[i].box.x
              << " Y: " << filtered_detections[i].box.y
              << " W: " << filtered_detections[i].box.width
              << " H: " << filtered_detections[i].box.height;
      cv::rectangle(frame, filtered_detections[i].box, cv::Scalar(255, 0, 0),
                    2);

      cv::putText(
          frame,
          "#" + std::to_string(filtered_detections[i].class_id) + " at " +
              std::to_string(filtered_detections[i].confidence) + " confidence",
          cv::Point(filtered_detections[i].box.x, filtered_detections[i].box.y),
          cv::FONT_HERSHEY_COMPLEX, 1.0, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
    }
    cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
    cv::imshow("yolo", frame);
    cv::waitKey(10);
  }

  return filtered_detections;
};

}  // namespace vision
}  // namespace y2023
